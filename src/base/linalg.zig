//! Small dense linear algebra on statically sized storage.
//!
//! Sized for the nonlinear systems of vehicle sizing (a few to a few dozen unknowns). The
//! matrix is a fixed `max_unknowns` square array of which the leading n x n block is used,
//! so solving never allocates.

const std = @import("std");
const assert = std.debug.assert;
const limits = @import("limits.zig");
const Rng = @import("rng.zig").Rng;

pub const max_dim: u32 = limits.max_unknowns;

pub const Mat = [max_dim][max_dim]f64;
pub const Vec = [max_dim]f64;

pub const SolveResult = struct {
    /// True when the matrix is numerically singular. `b` then holds no usable solution.
    singular: bool,
    /// min|pivot| / max|pivot|. A cheap conditioning indicator, not a rigorous bound:
    /// values near machine epsilon (1e-16) warn about an almost-singular system.
    pivot_ratio: f64,
};

/// Pivots smaller than this fraction of the largest matrix entry count as zero.
pub const singular_tolerance: f64 = 1e-13;

/// Solves A x = b for the leading n x n block, by LU decomposition with partial pivoting.
/// `a` is destroyed; the solution overwrites the first n entries of `b`.
pub fn solve(n: u32, a: *Mat, b: *Vec) SolveResult {
    assert(n >= 1 and n <= max_dim);
    assertFinite(n, a, b);

    const scale = maxAbs(n, a);
    if (scale == 0.0) return .{ .singular = true, .pivot_ratio = 0.0 };

    var min_pivot: f64 = std.math.inf(f64);
    var max_pivot: f64 = 0.0;
    for (0..n) |k| {
        const p = pivotRow(n, a, k);
        const pivot_abs = @abs(a[p][k]);
        if (pivot_abs <= singular_tolerance * scale) {
            return .{ .singular = true, .pivot_ratio = pivot_abs / @max(max_pivot, pivot_abs) };
        }
        min_pivot = @min(min_pivot, pivot_abs);
        max_pivot = @max(max_pivot, pivot_abs);
        swapRows(n, a, b, k, p);
        eliminate(n, a, b, k);
    }
    backSubstitute(n, a, b);

    for (b[0..n]) |value| {
        if (!std.math.isFinite(value)) return .{ .singular = true, .pivot_ratio = 0.0 };
    }
    return .{ .singular = false, .pivot_ratio = min_pivot / max_pivot };
}

/// out = A x for the leading n x n block.
pub fn matVec(n: u32, a: *const Mat, x: *const Vec, out: *Vec) void {
    assert(n >= 1 and n <= max_dim);
    for (0..n) |i| {
        var acc: f64 = 0.0;
        for (0..n) |j| acc += a[i][j] * x[j];
        out[i] = acc;
    }
}

/// Maximum norm of the first n entries.
pub fn normInf(n: u32, v: *const Vec) f64 {
    assert(n <= max_dim);
    var m: f64 = 0.0;
    for (v[0..n]) |x| m = @max(m, @abs(x));
    return m;
}

fn maxAbs(n: u32, a: *const Mat) f64 {
    var m: f64 = 0.0;
    for (0..n) |i| {
        for (0..n) |j| m = @max(m, @abs(a[i][j]));
    }
    return m;
}

fn assertFinite(n: u32, a: *const Mat, b: *const Vec) void {
    for (0..n) |i| {
        assert(std.math.isFinite(b[i]));
        for (0..n) |j| assert(std.math.isFinite(a[i][j]));
    }
}

fn pivotRow(n: u32, a: *const Mat, k: usize) usize {
    var best = k;
    for (k + 1..n) |i| {
        if (@abs(a[i][k]) > @abs(a[best][k])) best = i;
    }
    return best;
}

fn swapRows(n: u32, a: *Mat, b: *Vec, k: usize, p: usize) void {
    if (p == k) return;
    for (0..n) |j| std.mem.swap(f64, &a[k][j], &a[p][j]);
    std.mem.swap(f64, &b[k], &b[p]);
}

fn eliminate(n: u32, a: *Mat, b: *Vec, k: usize) void {
    for (k + 1..n) |i| {
        const factor = a[i][k] / a[k][k];
        a[i][k] = 0.0;
        for (k + 1..n) |j| a[i][j] -= factor * a[k][j];
        b[i] -= factor * b[k];
    }
}

fn backSubstitute(n: u32, a: *const Mat, b: *Vec) void {
    var i: usize = n;
    while (i > 0) {
        i -= 1;
        var acc = b[i];
        for (i + 1..n) |j| acc -= a[i][j] * b[j];
        b[i] = acc / a[i][i];
    }
}

fn zeroMat() Mat {
    return @splat(@splat(0.0));
}

test "identity system returns the right-hand side" {
    var a = zeroMat();
    var b: Vec = @splat(0.0);
    for (0..4) |i| {
        a[i][i] = 1.0;
        b[i] = @floatFromInt(i + 1);
    }
    const r = solve(4, &a, &b);
    try std.testing.expect(!r.singular);
    try std.testing.expectApproxEqAbs(@as(f64, 1.0), r.pivot_ratio, 1e-15);
    for (0..4) |i| try std.testing.expectEqual(@as(f64, @floatFromInt(i + 1)), b[i]);
}

test "classic 3x3 textbook system" {
    var a = zeroMat();
    a[0][0..3].* = .{ 2, 1, -1 };
    a[1][0..3].* = .{ -3, -1, 2 };
    a[2][0..3].* = .{ -2, 1, 2 };
    var b: Vec = @splat(0.0);
    b[0..3].* = .{ 8, -11, -3 };
    const r = solve(3, &a, &b);
    try std.testing.expect(!r.singular);
    try std.testing.expectApproxEqAbs(@as(f64, 2.0), b[0], 1e-12);
    try std.testing.expectApproxEqAbs(@as(f64, 3.0), b[1], 1e-12);
    try std.testing.expectApproxEqAbs(@as(f64, -1.0), b[2], 1e-12);
}

test "a zero on the diagonal needs pivoting" {
    var a = zeroMat();
    a[0][0..2].* = .{ 0, 1 };
    a[1][0..2].* = .{ 1, 0 };
    var b: Vec = @splat(0.0);
    b[0..2].* = .{ 2, 3 };
    const r = solve(2, &a, &b);
    try std.testing.expect(!r.singular);
    try std.testing.expectEqual(@as(f64, 3.0), b[0]);
    try std.testing.expectEqual(@as(f64, 2.0), b[1]);
}

test "singular matrices are detected" {
    var a = zeroMat();
    a[0][0..3].* = .{ 1, 2, 3 };
    a[1][0..3].* = .{ 2, 4, 6 };
    a[2][0..3].* = .{ 0, 1, 1 };
    var b: Vec = @splat(1.0);
    try std.testing.expect(solve(3, &a, &b).singular);

    var zero = zeroMat();
    var rhs: Vec = @splat(1.0);
    try std.testing.expect(solve(2, &zero, &rhs).singular);
}

test "random diagonally dominant systems are solved accurately" {
    var rng = Rng.init(31337);
    for (0..500) |_| {
        const n = 1 + rng.below(max_dim);
        var a = zeroMat();
        var x_true: Vec = @splat(0.0);
        for (0..n) |i| {
            var row_sum: f64 = 0.0;
            for (0..n) |j| {
                a[i][j] = rng.range(-1.0, 1.0);
                row_sum += @abs(a[i][j]);
            }
            a[i][i] += row_sum + 0.5;
            x_true[i] = rng.range(-10.0, 10.0);
        }
        var b: Vec = @splat(0.0);
        matVec(n, &a, &x_true, &b);

        const r = solve(n, &a, &b);
        try std.testing.expect(!r.singular);
        for (0..n) |i| try std.testing.expectApproxEqAbs(x_true[i], b[i], 1e-9);
    }
}

test "residual of the solution is tiny for general random systems" {
    var rng = Rng.init(555);
    for (0..300) |_| {
        const n = 2 + rng.below(10);
        var a = zeroMat();
        var b: Vec = @splat(0.0);
        for (0..n) |i| {
            b[i] = rng.range(-5.0, 5.0);
            for (0..n) |j| a[i][j] = rng.range(-1.0, 1.0);
        }
        const a_copy = a;
        const b_copy = b;
        const r = solve(n, &a, &b);
        if (r.singular) continue;
        var ax: Vec = @splat(0.0);
        matVec(n, &a_copy, &b, &ax);
        var worst: f64 = 0.0;
        for (0..n) |i| worst = @max(worst, @abs(ax[i] - b_copy[i]));
        // Residual scales with the condition number; the pivot ratio bounds it loosely.
        try std.testing.expect(worst < 1e-8 / @max(r.pivot_ratio, 1e-6));
    }
}

test "normInf" {
    var v: Vec = @splat(0.0);
    v[0..3].* = .{ 1.0, -7.5, 3.0 };
    try std.testing.expectEqual(@as(f64, 7.5), normInf(3, &v));
    try std.testing.expectEqual(@as(f64, 1.0), normInf(1, &v));
}
