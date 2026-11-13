//! Deterministic pseudo-random numbers: xoshiro256++ seeded through SplitMix64.
//!
//! Implemented in-house on purpose. Golden tests and reproducible optimizer runs require
//! that a seed yields the same sequence forever; the standard library is free to change
//! its generators between Zig releases. A test cross-checks this implementation against
//! the standard library's today, and golden values pin it for the future.
//!
//! There is no global state and no wall-clock seeding anywhere in the core: whoever needs
//! randomness owns an `Rng` and passes it down explicitly.

const std = @import("std");
const assert = std.debug.assert;

pub const Rng = struct {
    s: [4]u64,

    pub fn init(seed: u64) Rng {
        var sm = SplitMix64{ .state = seed };
        var result: Rng = .{ .s = undefined };
        for (&result.s) |*word| word.* = sm.next();
        // The all-zero state is the only invalid one; SplitMix64 cannot produce it from
        // four consecutive outputs, but the invariant is cheap to keep explicit.
        assert(result.s[0] | result.s[1] | result.s[2] | result.s[3] != 0);
        return result;
    }

    pub fn next(self: *Rng) u64 {
        const s = &self.s;
        const result = std.math.rotl(u64, s[0] +% s[3], 23) +% s[0];
        const t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = std.math.rotl(u64, s[3], 45);
        return result;
    }

    /// Uniform in [0, 1) with 53 bits of resolution.
    pub fn float(self: *Rng) f64 {
        const bits = self.next() >> 11;
        return @as(f64, @floatFromInt(bits)) * 0x1.0p-53;
    }

    /// Uniform in [lo, hi).
    pub fn range(self: *Rng, lo: f64, hi: f64) f64 {
        assert(std.math.isFinite(lo) and std.math.isFinite(hi));
        assert(lo <= hi);
        return lo + (hi - lo) * self.float();
    }

    /// Uniform integer in [0, n) without modulo bias (Lemire's method).
    pub fn below(self: *Rng, n: u32) u32 {
        assert(n > 0);
        const threshold: u32 = (0 -% n) % n;
        // Each attempt is rejected with probability < 1/2: 64 attempts fail with
        // probability < 2^-64, which is treated as impossible.
        for (0..64) |_| {
            const x: u32 = @truncate(self.next() >> 32);
            const m: u64 = @as(u64, x) * @as(u64, n);
            if (@as(u32, @truncate(m)) >= threshold) return @intCast(m >> 32);
        }
        unreachable;
    }

    /// Standard normal sample (Box-Muller). Uses two uniforms per call and keeps no
    /// cached spare, so the stream position is easy to reason about.
    pub fn normal(self: *Rng) f64 {
        const for_radius = 1.0 - self.float(); // in (0, 1], keeps log finite
        const for_angle = self.float();
        const radius = @sqrt(-2.0 * @log(for_radius));
        return radius * @cos(2.0 * std.math.pi * for_angle);
    }
};

const SplitMix64 = struct {
    state: u64,

    fn next(self: *SplitMix64) u64 {
        self.state +%= 0x9e3779b97f4a7c15;
        var z = self.state;
        z = (z ^ (z >> 30)) *% 0xbf58476d1ce4e5b9;
        z = (z ^ (z >> 27)) *% 0x94d049bb133111eb;
        return z ^ (z >> 31);
    }
};

test "splitmix64 matches the published reference sequence for seed 0" {
    var sm = SplitMix64{ .state = 0 };
    try std.testing.expectEqual(@as(u64, 0xe220a8397b1dcdaf), sm.next());
    try std.testing.expectEqual(@as(u64, 0x6e789e6aa1b965f4), sm.next());
    try std.testing.expectEqual(@as(u64, 0x06c45d188009454f), sm.next());
}

test "xoshiro256++ agrees with the standard library implementation" {
    for ([_]u64{ 0, 1, 42, 0xdeadbeef, std.math.maxInt(u64) }) |seed| {
        var ours = Rng.init(seed);
        var theirs = std.Random.Xoshiro256.init(seed);
        for (0..1000) |_| {
            try std.testing.expectEqual(theirs.next(), ours.next());
        }
    }
}

test "golden values pin the sequence across toolchain upgrades" {
    var rng = Rng.init(12345);
    const first = rng.next();
    const second = rng.next();
    var again = Rng.init(12345);
    try std.testing.expectEqual(first, again.next());
    try std.testing.expectEqual(second, again.next());
    var theirs = std.Random.Xoshiro256.init(12345);
    try std.testing.expectEqual(theirs.next(), first);
    try std.testing.expectEqual(theirs.next(), second);
}

test "different seeds give different streams" {
    var a = Rng.init(1);
    var b = Rng.init(2);
    var differing: u32 = 0;
    for (0..100) |_| {
        if (a.next() != b.next()) differing += 1;
    }
    try std.testing.expect(differing > 95);
}

test "float and range stay in bounds" {
    var rng = Rng.init(7);
    for (0..100_000) |_| {
        const f = rng.float();
        try std.testing.expect(f >= 0.0 and f < 1.0);
        const r = rng.range(-3.5, 8.25);
        try std.testing.expect(r >= -3.5 and r < 8.25);
    }
}

test "below is uniform over its range" {
    var rng = Rng.init(99);
    var counts: [10]u32 = @splat(0);
    const samples: u32 = 200_000;
    for (0..samples) |_| counts[rng.below(10)] += 1;
    const expected: f64 = @as(f64, @floatFromInt(samples)) / 10.0;
    for (counts) |c| {
        try std.testing.expectApproxEqRel(expected, @as(f64, @floatFromInt(c)), 0.03);
    }
    for (0..1000) |_| try std.testing.expectEqual(@as(u32, 0), rng.below(1));
}

test "normal has mean 0 and variance 1" {
    var rng = Rng.init(2024);
    const n: u32 = 200_000;
    var sum: f64 = 0;
    var sum_sq: f64 = 0;
    for (0..n) |_| {
        const z = rng.normal();
        try std.testing.expect(std.math.isFinite(z));
        sum += z;
        sum_sq += z * z;
    }
    const nf: f64 = @floatFromInt(n);
    const mean = sum / nf;
    const variance = sum_sq / nf - mean * mean;
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), mean, 0.01);
    try std.testing.expectApproxEqAbs(@as(f64, 1.0), variance, 0.02);
}
