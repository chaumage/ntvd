//! Piecewise-linear lookup tables for measured or simulated data.
//!
//! This is how a simple analytic model gets replaced by wind-tunnel, towing-tank or CFD
//! results without touching the rest of the architecture. Two rules shape the design:
//!   1. Tables are validated once, at construction (finite, strictly increasing axes).
//!   2. A lookup outside the measured range never extrapolates silently: the value is
//!      clamped to the nearest edge and the result is flagged `beyond_envelope`.

const std = @import("std");
const assert = std.debug.assert;
const Validity = @import("validity.zig").Validity;
const Rng = @import("rng.zig").Rng;

pub const Lookup = struct {
    value: f64,
    validity: Validity,
};

pub const TableError = error{
    TooFewPoints,
    TooManyPoints,
    LengthMismatch,
    NotFinite,
    NotStrictlyIncreasing,
};

/// Location of x on an axis: interval index i (segment [i, i+1]), fraction t in [0, 1]
/// inside it, and whether x had to be clamped into the axis range.
const Segment = struct { i: u32, t: f64, clamped: bool };

fn locate(axis: []const f64, x: f64) Segment {
    assert(axis.len >= 2);
    assert(std.math.isFinite(x));
    const last: u32 = @intCast(axis.len - 1);
    if (x < axis[0]) return .{ .i = 0, .t = 0.0, .clamped = true };
    if (x > axis[last]) return .{ .i = last - 1, .t = 1.0, .clamped = true };

    // Binary search keeping axis[lo] <= x <= axis[hi]. Halving a 32-bit range cannot
    // take more than 32 steps.
    var lo: u32 = 0;
    var hi: u32 = last;
    for (0..32) |_| {
        if (hi - lo <= 1) break;
        const mid = lo + (hi - lo) / 2;
        if (axis[mid] <= x) lo = mid else hi = mid;
    } else unreachable;

    const t = (x - axis[lo]) / (axis[hi] - axis[lo]);
    assert(t >= 0.0 and t <= 1.0);
    return .{ .i = lo, .t = t, .clamped = false };
}

fn validateAxis(axis: []const f64) TableError!void {
    for (axis) |v| {
        if (!std.math.isFinite(v)) return error.NotFinite;
    }
    for (axis[1..], 0..) |v, i| {
        if (v <= axis[i]) return error.NotStrictlyIncreasing;
    }
}

fn blend(a: f64, b: f64, t: f64) f64 {
    return (1.0 - t) * a + t * b;
}

/// y = f(x) from `capacity` points at most.
pub fn Table1D(comptime capacity: u32) type {
    comptime assert(capacity >= 2);

    return struct {
        const Self = @This();

        xs: [capacity]f64,
        ys: [capacity]f64,
        len: u32,

        pub fn init(xs: []const f64, ys: []const f64) TableError!Self {
            if (xs.len != ys.len) return error.LengthMismatch;
            if (xs.len < 2) return error.TooFewPoints;
            if (xs.len > capacity) return error.TooManyPoints;
            try validateAxis(xs);
            for (ys) |v| {
                if (!std.math.isFinite(v)) return error.NotFinite;
            }
            var t: Self = .{ .xs = @splat(0.0), .ys = @splat(0.0), .len = @intCast(xs.len) };
            @memcpy(t.xs[0..xs.len], xs);
            @memcpy(t.ys[0..ys.len], ys);
            return t;
        }

        pub fn lookup(self: *const Self, x: f64) Lookup {
            const seg = locate(self.xs[0..self.len], x);
            const y = blend(self.ys[seg.i], self.ys[seg.i + 1], seg.t);
            return .{ .value = y, .validity = if (seg.clamped) .beyond_envelope else .valid };
        }

        pub fn xMin(self: *const Self) f64 {
            return self.xs[0];
        }

        pub fn xMax(self: *const Self) f64 {
            return self.xs[self.len - 1];
        }
    };
}

/// z = f(x, y) on a rectilinear grid, bilinear interpolation. Values are stored with x as
/// the slow index: z[i * ny + j] belongs to (xs[i], ys[j]).
pub fn Table2D(comptime nx_capacity: u32, comptime ny_capacity: u32) type {
    comptime assert(nx_capacity >= 2 and ny_capacity >= 2);

    return struct {
        const Self = @This();

        xs: [nx_capacity]f64,
        ys: [ny_capacity]f64,
        zs: [nx_capacity * ny_capacity]f64,
        nx: u32,
        ny: u32,

        pub fn init(xs: []const f64, ys: []const f64, zs: []const f64) TableError!Self {
            if (xs.len < 2 or ys.len < 2) return error.TooFewPoints;
            if (xs.len > nx_capacity or ys.len > ny_capacity) return error.TooManyPoints;
            if (zs.len != xs.len * ys.len) return error.LengthMismatch;
            try validateAxis(xs);
            try validateAxis(ys);
            for (zs) |v| {
                if (!std.math.isFinite(v)) return error.NotFinite;
            }
            var t: Self = .{
                .xs = @splat(0.0),
                .ys = @splat(0.0),
                .zs = @splat(0.0),
                .nx = @intCast(xs.len),
                .ny = @intCast(ys.len),
            };
            @memcpy(t.xs[0..xs.len], xs);
            @memcpy(t.ys[0..ys.len], ys);
            @memcpy(t.zs[0..zs.len], zs);
            return t;
        }

        pub fn lookup(self: *const Self, x: f64, y: f64) Lookup {
            const sx = locate(self.xs[0..self.nx], x);
            const sy = locate(self.ys[0..self.ny], y);
            const z00 = self.at(sx.i, sy.i);
            const z01 = self.at(sx.i, sy.i + 1);
            const z10 = self.at(sx.i + 1, sy.i);
            const z11 = self.at(sx.i + 1, sy.i + 1);
            const value = blend(blend(z00, z01, sy.t), blend(z10, z11, sy.t), sx.t);
            const clamped = sx.clamped or sy.clamped;
            return .{ .value = value, .validity = if (clamped) .beyond_envelope else .valid };
        }

        fn at(self: *const Self, i: u32, j: u32) f64 {
            assert(i < self.nx and j < self.ny);
            return self.zs[i * self.ny + j];
        }
    };
}

test "1D: knots are reproduced exactly" {
    const T = Table1D(8);
    const t = try T.init(&.{ 0.0, 1.0, 3.0, 4.0 }, &.{ 10.0, 20.0, 0.0, 5.0 });
    for ([_]f64{ 0.0, 1.0, 3.0, 4.0 }, [_]f64{ 10.0, 20.0, 0.0, 5.0 }) |x, y| {
        const r = t.lookup(x);
        try std.testing.expectEqual(y, r.value);
        try std.testing.expectEqual(Validity.valid, r.validity);
    }
    try std.testing.expectEqual(@as(f64, 0.0), t.xMin());
    try std.testing.expectEqual(@as(f64, 4.0), t.xMax());
}

test "1D: linear data is reproduced between the knots" {
    const T = Table1D(16);
    var xs: [10]f64 = undefined;
    var ys: [10]f64 = undefined;
    for (&xs, &ys, 0..) |*x, *y, i| {
        x.* = -2.0 + 0.7 * @as(f64, @floatFromInt(i * i + i));
        y.* = 3.0 * x.* - 1.0;
    }
    const t = try T.init(&xs, &ys);
    var rng = Rng.init(1);
    for (0..10_000) |_| {
        const x = rng.range(xs[0], xs[9]);
        const r = t.lookup(x);
        try std.testing.expectEqual(Validity.valid, r.validity);
        try std.testing.expectApproxEqAbs(3.0 * x - 1.0, r.value, 1e-9);
    }
}

test "1D: midpoint interpolation" {
    const t = try Table1D(4).init(&.{ 0.0, 2.0 }, &.{ 1.0, 5.0 });
    try std.testing.expectApproxEqAbs(@as(f64, 3.0), t.lookup(1.0).value, 1e-15);
    try std.testing.expectApproxEqAbs(@as(f64, 1.5), t.lookup(0.25).value, 1e-15);
}

test "1D: out of range clamps and flags, never extrapolates" {
    const t = try Table1D(4).init(&.{ 1.0, 2.0, 3.0 }, &.{ 5.0, 6.0, 9.0 });
    const below = t.lookup(-100.0);
    try std.testing.expectEqual(@as(f64, 5.0), below.value);
    try std.testing.expectEqual(Validity.beyond_envelope, below.validity);
    const above = t.lookup(1e9);
    try std.testing.expectEqual(@as(f64, 9.0), above.value);
    try std.testing.expectEqual(Validity.beyond_envelope, above.validity);
}

test "1D: monotone data gives a monotone, continuous interpolant" {
    const T = Table1D(32);
    var rng = Rng.init(77);
    for (0..200) |_| {
        var xs: [32]f64 = undefined;
        var ys: [32]f64 = undefined;
        var x: f64 = rng.range(-5.0, 5.0);
        var y: f64 = rng.range(-5.0, 5.0);
        const n = 2 + rng.below(31);
        for (0..n) |i| {
            xs[i] = x;
            ys[i] = y;
            x += rng.range(0.01, 2.0);
            y += rng.range(0.0, 3.0);
        }
        const t = try T.init(xs[0..n], ys[0..n]);
        var prev = t.lookup(xs[0]).value;
        const steps: u32 = 400;
        for (1..steps + 1) |s| {
            const frac = @as(f64, @floatFromInt(s)) / @as(f64, @floatFromInt(steps));
            const cur = t.lookup(xs[0] + frac * (xs[n - 1] - xs[0])).value;
            try std.testing.expect(cur >= prev - 1e-12);
            prev = cur;
        }
    }
}

test "1D: construction rejects bad data" {
    const T = Table1D(3);
    try std.testing.expectError(error.TooFewPoints, T.init(&.{1.0}, &.{1.0}));
    try std.testing.expectError(error.LengthMismatch, T.init(&.{ 1.0, 2.0 }, &.{1.0}));
    try std.testing.expectError(
        error.TooManyPoints,
        T.init(&.{ 1.0, 2.0, 3.0, 4.0 }, &.{ 1.0, 2.0, 3.0, 4.0 }),
    );
    try std.testing.expectError(
        error.NotStrictlyIncreasing,
        T.init(&.{ 1.0, 1.0, 2.0 }, &.{ 1.0, 2.0, 3.0 }),
    );
    try std.testing.expectError(
        error.NotStrictlyIncreasing,
        T.init(&.{ 2.0, 1.0 }, &.{ 1.0, 2.0 }),
    );
    try std.testing.expectError(
        error.NotFinite,
        T.init(&.{ 1.0, std.math.nan(f64) }, &.{ 1.0, 2.0 }),
    );
    try std.testing.expectError(
        error.NotFinite,
        T.init(&.{ 1.0, 2.0 }, &.{ 1.0, std.math.inf(f64) }),
    );
}

test "2D: a bilinear function is reproduced exactly" {
    const T = Table2D(6, 5);
    const xs = [_]f64{ -1.0, 0.0, 0.5, 2.0, 5.0 };
    const ys = [_]f64{ 10.0, 11.0, 14.0, 20.0 };
    var zs: [xs.len * ys.len]f64 = undefined;
    for (xs, 0..) |x, i| {
        for (ys, 0..) |y, j| zs[i * ys.len + j] = bilinear(x, y);
    }
    const t = try T.init(&xs, &ys, &zs);
    var rng = Rng.init(5);
    for (0..10_000) |_| {
        const x = rng.range(-1.0, 5.0);
        const y = rng.range(10.0, 20.0);
        const r = t.lookup(x, y);
        try std.testing.expectEqual(Validity.valid, r.validity);
        try std.testing.expectApproxEqAbs(bilinear(x, y), r.value, 1e-9);
    }
}

fn bilinear(x: f64, y: f64) f64 {
    return 1.5 + 2.0 * x - 0.25 * y + 0.125 * x * y;
}

test "2D: clamping on either axis is flagged" {
    const t = try Table2D(2, 2).init(&.{ 0.0, 1.0 }, &.{ 0.0, 1.0 }, &.{ 0.0, 1.0, 2.0, 3.0 });
    try std.testing.expectEqual(Validity.valid, t.lookup(0.5, 0.5).validity);
    try std.testing.expectEqual(Validity.beyond_envelope, t.lookup(-0.1, 0.5).validity);
    try std.testing.expectEqual(Validity.beyond_envelope, t.lookup(0.5, 1.1).validity);
    // Corner value after clamping both axes.
    try std.testing.expectEqual(@as(f64, 3.0), t.lookup(9.0, 9.0).value);
    // Slow index is x: z[1 * 2 + 0] is (x=1, y=0).
    try std.testing.expectEqual(@as(f64, 2.0), t.lookup(1.0, 0.0).value);
}

test "2D: construction rejects bad data" {
    const T = Table2D(3, 3);
    const ok = [_]f64{ 0.0, 1.0 };
    try std.testing.expectError(error.TooFewPoints, T.init(&.{0.0}, &ok, &.{ 1.0, 2.0 }));
    try std.testing.expectError(error.LengthMismatch, T.init(&ok, &ok, &.{ 1.0, 2.0, 3.0 }));
    try std.testing.expectError(
        error.NotStrictlyIncreasing,
        T.init(&ok, &.{ 1.0, 0.0 }, &.{ 1.0, 2.0, 3.0, 4.0 }),
    );
    try std.testing.expectError(
        error.NotFinite,
        T.init(&ok, &ok, &.{ 1.0, 2.0, std.math.nan(f64), 4.0 }),
    );
    const eight: [8]f64 = @splat(0.0);
    try std.testing.expectError(
        error.TooManyPoints,
        T.init(&.{ 0.0, 1.0, 2.0, 3.0 }, &ok, &eight),
    );
}
