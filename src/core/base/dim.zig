//! Physical dimensions as integer exponents of the SI base units.
//!
//! The core computes in SI everywhere. Dimensions exist for one purpose: when two ports
//! are connected, the model compiler checks that both sides carry the same dimension, so
//! wiring a force into a speed input is rejected before any number is computed.
//! Angle is tracked as its own pseudo-dimension (radian) because confusing an angle with a
//! plain ratio is a classic source of silent bugs, even though it is dimensionless in SI.

const std = @import("std");

pub const Dim = struct {
    m: i8 = 0,
    kg: i8 = 0,
    s: i8 = 0,
    k: i8 = 0,
    a: i8 = 0,
    rad: i8 = 0,

    pub fn eql(a: Dim, b: Dim) bool {
        return a.m == b.m and a.kg == b.kg and a.s == b.s and
            a.k == b.k and a.a == b.a and a.rad == b.rad;
    }

    pub fn mul(a: Dim, b: Dim) Dim {
        return .{
            .m = a.m + b.m,
            .kg = a.kg + b.kg,
            .s = a.s + b.s,
            .k = a.k + b.k,
            .a = a.a + b.a,
            .rad = a.rad + b.rad,
        };
    }

    pub fn div(a: Dim, b: Dim) Dim {
        return .{
            .m = a.m - b.m,
            .kg = a.kg - b.kg,
            .s = a.s - b.s,
            .k = a.k - b.k,
            .a = a.a - b.a,
            .rad = a.rad - b.rad,
        };
    }

    pub fn pow(a: Dim, n: i8) Dim {
        return .{
            .m = a.m * n,
            .kg = a.kg * n,
            .s = a.s * n,
            .k = a.k * n,
            .a = a.a * n,
            .rad = a.rad * n,
        };
    }

    /// Human readable form for diagnostics, e.g. "kg*m*s^-2". Dimensionless prints "1".
    pub fn format(self: Dim, w: *std.Io.Writer) std.Io.Writer.Error!void {
        const names = [_][]const u8{ "m", "kg", "s", "K", "A", "rad" };
        const exps = [_]i8{ self.m, self.kg, self.s, self.k, self.a, self.rad };
        var first = true;
        for (names, exps) |name, e| {
            if (e == 0) continue;
            if (!first) try w.writeAll("*");
            first = false;
            try w.writeAll(name);
            if (e != 1) try w.print("^{d}", .{e});
        }
        if (first) try w.writeAll("1");
    }
};

pub const dimensionless: Dim = .{};
pub const length: Dim = .{ .m = 1 };
pub const mass: Dim = .{ .kg = 1 };
pub const time: Dim = .{ .s = 1 };
pub const temperature: Dim = .{ .k = 1 };
pub const current: Dim = .{ .a = 1 };
pub const angle: Dim = .{ .rad = 1 };

pub const area: Dim = length.pow(2);
pub const volume: Dim = length.pow(3);
pub const speed: Dim = length.div(time);
pub const acceleration: Dim = speed.div(time);
pub const force: Dim = mass.mul(acceleration);
pub const moment: Dim = force.mul(length);
pub const inertia: Dim = mass.mul(area);
pub const mass_flow: Dim = mass.div(time);
pub const pressure: Dim = force.div(area);
pub const energy: Dim = force.mul(length);
pub const power: Dim = energy.div(time);
pub const density: Dim = mass.div(volume);
pub const dynamic_viscosity: Dim = pressure.mul(time);
pub const angular_rate: Dim = angle.div(time);
pub const angular_momentum: Dim = inertia.mul(angular_rate);
pub const voltage: Dim = power.div(current);

test "derived dimensions are consistent" {
    try std.testing.expect(force.eql(mass.mul(acceleration)));
    try std.testing.expect(force.eql(.{ .kg = 1, .m = 1, .s = -2 }));
    try std.testing.expect(power.eql(force.mul(speed)));
    try std.testing.expect(energy.eql(.{ .kg = 1, .m = 2, .s = -2 }));
    try std.testing.expect(pressure.eql(.{ .kg = 1, .m = -1, .s = -2 }));
    try std.testing.expect(density.eql(.{ .kg = 1, .m = -3 }));
    try std.testing.expect(moment.eql(.{ .kg = 1, .m = 2, .s = -2 }));
    try std.testing.expect(inertia.eql(.{ .kg = 1, .m = 2 }));
    try std.testing.expect(mass_flow.eql(.{ .kg = 1, .s = -1 }));
    try std.testing.expect(angular_momentum.eql(.{ .kg = 1, .m = 2, .s = -1, .rad = 1 }));
    try std.testing.expect(voltage.eql(.{ .kg = 1, .m = 2, .s = -3, .a = -1 }));
}

test "dynamic pressure times area is a force" {
    const q_times_s = density.mul(speed.pow(2)).mul(area);
    try std.testing.expect(q_times_s.eql(force));
}

test "mul and div are inverse, pow composes" {
    const dims = [_]Dim{ length, mass, speed, force, density, angle, voltage };
    for (dims) |a| {
        for (dims) |b| {
            try std.testing.expect(a.mul(b).div(b).eql(a));
            try std.testing.expect(a.mul(b).eql(b.mul(a)));
        }
        try std.testing.expect(a.pow(2).eql(a.mul(a)));
        try std.testing.expect(a.pow(0).eql(dimensionless));
    }
    try std.testing.expect(!length.eql(angle));
}

test "format for diagnostics" {
    var buf: [64]u8 = undefined;
    const f = try std.fmt.bufPrint(&buf, "{f}", .{force});
    try std.testing.expectEqualStrings("m*kg*s^-2", f);
    const d = try std.fmt.bufPrint(&buf, "{f}", .{dimensionless});
    try std.testing.expectEqualStrings("1", d);
    const a = try std.fmt.bufPrint(&buf, "{f}", .{angle});
    try std.testing.expectEqualStrings("rad", a);
}
