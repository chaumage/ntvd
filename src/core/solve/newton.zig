//! Damped Newton iteration on a small system of nonlinear residuals.
//!
//! The residual is supplied as a comptime function
//! `fn (ctx, x: []f64, r: []f64) Validity`; it fills `r` and reports how far the probe is
//! from the model's calibrated envelope. The Jacobian is finite-differenced forward, the
//! linear correction uses `base.linalg.solve`, and a step is accepted only when it reduces
//! the maximum-norm residual. Everything is stack-bounded by `base.limits.max_unknowns` and
//! allocation-free, so the same call is deterministic bit for bit.
//!
//! On failure the iterate `x` and residual `r` still hold the best point found; only accepted
//! steps are written back. A `.invalid` residual (a bound violation in the caller's problem)
//! ends the solve with `outside_envelope`; `.beyond_envelope` is carried to the outcome but
//! does not stop the search.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");

const linalg = base.linalg;
const Validity = base.Validity;

pub const Options = struct {
    max_iterations: u16 = 40,
    tolerance: f64 = 1e-9,
    /// Largest absolute correction applied to one unknown before damping.
    step_limit: f64 = 10.0,
    /// Initial fraction of the correction; halved up to six times without progress.
    damping: f64 = 1.0,
};

pub const Status = enum {
    converged,
    max_iterations,
    singular,
    outside_envelope,
    diverged,
};

pub const Outcome = struct {
    status: Status = .max_iterations,
    iterations: u16 = 0,
    residual_norm: f64 = std.math.inf(f64),
    pivot_ratio: f64 = 1.0,
    validity: Validity = .valid,
};

const max_backtracks: u8 = 6;
const finite_hint: f64 = 1.0;

/// Solves `r(x) = 0` in place. `x` starts at the initial guess and ends at the best point.
pub fn solve(comptime residual: anytype, ctx: anytype, x: []f64, r: []f64, opts: Options) Outcome {
    const n: u32 = @intCast(x.len);
    assert(n >= 1 and n <= linalg.max_dim);
    assert(r.len == x.len);
    assert(opts.tolerance > 0.0 and std.math.isFinite(opts.tolerance));
    assert(opts.step_limit > 0.0 and std.math.isFinite(opts.step_limit));
    assert(opts.damping > 0.0 and opts.damping <= 1.0 and std.math.isFinite(opts.damping));

    var outcome = Outcome{};
    const initial = evaluate(residual, ctx, x, r);
    switch (initial) {
        .ok => |v| outcome.validity = v,
        else => {
            outcome.status = failureStatus(initial);
            return outcome;
        },
    }
    outcome.residual_norm = norm(r);
    if (outcome.residual_norm <= opts.tolerance) {
        outcome.status = .converged;
        return outcome;
    }

    var a: linalg.Mat = undefined;
    var step: linalg.Vec = undefined;
    var probe_r: linalg.Vec = undefined;
    var trial: Trial = undefined;
    while (outcome.iterations < opts.max_iterations) {
        outcome.iterations += 1;
        const checked = jacobian(residual, ctx, x, r, n, &a, &probe_r);
        switch (checked) {
            .ok => |v| outcome.validity = outcome.validity.worst(v),
            else => {
                outcome.status = failureStatus(checked);
                return outcome;
            },
        }
        @memcpy(step[0..n], r[0..n]);
        for (step[0..n]) |*s| s.* = -s.*;
        const linear = linalg.solve(n, &a, &step);
        outcome.pivot_ratio = linear.pivot_ratio;
        if (linear.singular) {
            outcome.status = .singular;
            return outcome;
        }

        const attempt = advance(residual, ctx, x, n, &step, opts, outcome.residual_norm, &trial);
        switch (attempt) {
            .bad => {
                outcome.status = .diverged;
                return outcome;
            },
            .invalid, .no_progress => continue,
            .ok => |found| {
                @memcpy(x, trial.x[0..n]);
                @memcpy(r, trial.r[0..n]);
                outcome.residual_norm = found.norm;
                outcome.validity = outcome.validity.worst(found.validity);
                if (found.norm <= opts.tolerance) {
                    outcome.status = .converged;
                    return outcome;
                }
            },
        }
    }
    return outcome;
}

const Check = union(enum) {
    ok: Validity,
    /// A non-finite iterate or residual.
    bad,
    /// A bound violation reported by the residual.
    invalid,
};

/// Maps a failed residual check to its terminal status; callers pass a non-`.ok` check.
fn failureStatus(check: Check) Status {
    return switch (check) {
        .bad => .diverged,
        .invalid => .outside_envelope,
        .ok => unreachable,
    };
}

const Trial = struct {
    x: linalg.Vec = undefined,
    r: linalg.Vec = undefined,
};

fn evaluate(comptime residual: anytype, ctx: anytype, x: []const f64, r: []f64) Check {
    const validity = residual(ctx, x, r);
    if (!allFinite(r) or !allFinite(x)) return .bad;
    return switch (validity) {
        .invalid => .invalid,
        else => .{ .ok = validity },
    };
}

/// Forward differences with a backward fallback when a probe leaves the caller's bounds.
/// Column j is `(r(x + h e_j) - r(x)) / h`, or `(r(x) - r(x - h e_j)) / h` on fallback.
fn jacobian(
    comptime residual: anytype,
    ctx: anytype,
    x: []f64,
    r: []const f64,
    n: u32,
    a: *linalg.Mat,
    probe_r: *linalg.Vec,
) Check {
    const base_step = std.math.sqrt(std.math.floatEps(f64));
    var worst: Validity = .valid;
    for (0..n) |j| {
        const saved = x[j];
        const h = base_step * @max(finite_hint, @abs(saved));
        var backward = false;
        x[j] = saved + h;
        var checked = evaluate(residual, ctx, x, probe_r[0..n]);
        switch (checked) {
            .invalid => {
                backward = true;
                x[j] = saved - h;
                checked = evaluate(residual, ctx, x, probe_r[0..n]);
            },
            else => {},
        }
        x[j] = saved;
        switch (checked) {
            .bad => return .bad,
            .invalid => return .invalid,
            .ok => |v| worst = worst.worst(v),
        }
        for (0..n) |i| {
            a[i][j] = if (backward) (r[i] - probe_r[i]) / h else (probe_r[i] - r[i]) / h;
        }
    }
    return .{ .ok = worst };
}

const Advance = union(enum) {
    ok: struct { norm: f64, validity: Validity },
    bad,
    invalid,
    no_progress,
};

/// Clips the correction, then damp it until the residual norm drops.
fn advance(
    comptime residual: anytype,
    ctx: anytype,
    x: []const f64,
    n: u32,
    step: *const linalg.Vec,
    opts: Options,
    current_norm: f64,
    trial: *Trial,
) Advance {
    var damping = opts.damping;
    for (0..max_backtracks + 1) |_| {
        for (0..n) |i| {
            const clipped = std.math.clamp(step[i], -opts.step_limit, opts.step_limit);
            trial.x[i] = x[i] + damping * clipped;
        }
        if (!allFinite(trial.x[0..n])) return .bad;
        const checked = evaluate(residual, ctx, trial.x[0..n], trial.r[0..n]);
        switch (checked) {
            .bad => return .bad,
            .invalid => {},
            .ok => |v| {
                const trial_norm = norm(trial.r[0..n]);
                if (trial_norm < current_norm) {
                    return .{ .ok = .{ .norm = trial_norm, .validity = v } };
                }
            },
        }
        damping *= 0.5;
    }
    return .no_progress;
}

fn allFinite(values: []const f64) bool {
    for (values) |value| if (!std.math.isFinite(value)) return false;
    return true;
}

fn norm(values: []const f64) f64 {
    var result: f64 = 0.0;
    for (values) |value| result = @max(result, @abs(value));
    return result;
}

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

const Linear = struct {
    a: [2][2]f64 = .{ .{ 2.0, 1.0 }, .{ 1.0, 3.0 } },
    b: [2]f64 = .{ 5.0, 7.0 },
    validity: Validity = .valid,

    fn residual(self: *const Linear, x: []const f64, r: []f64) Validity {
        for (0..2) |i| {
            r[i] = -self.b[i];
            for (0..2) |j| r[i] += self.a[i][j] * x[j];
        }
        return self.validity;
    }
};

test "a linear system converges in one iteration" {
    const system = Linear{};
    var x = [_]f64{ 0.0, 0.0 };
    var r: [2]f64 = undefined;
    const out = solve(Linear.residual, &system, &x, &r, .{});

    try std.testing.expectEqual(Status.converged, out.status);
    try std.testing.expectEqual(@as(u16, 1), out.iterations);
    try std.testing.expect(out.residual_norm <= 1e-9);
    try std.testing.expectApproxEqAbs(@as(f64, 1.6), x[0], 1e-12);
    try std.testing.expectApproxEqAbs(@as(f64, 1.8), x[1], 1e-12);
    try std.testing.expectEqual(Validity.valid, out.validity);
}

const Rosenbrock = struct {
    fn residual(self: *const Rosenbrock, x: []const f64, r: []f64) Validity {
        _ = self;
        r[0] = 10.0 * (x[1] - x[0] * x[0]);
        r[1] = 1.0 - x[0];
        return .valid;
    }
};

test "a nonlinear residual reaches its root" {
    const system = Rosenbrock{};
    var x = [_]f64{ -1.2, 1.0 };
    var r: [2]f64 = undefined;
    const options: Options = .{ .max_iterations = 200, .tolerance = 1e-12 };
    const out = solve(Rosenbrock.residual, &system, &x, &r, options);

    try std.testing.expectEqual(Status.converged, out.status);
    try std.testing.expect(out.iterations < 40);
    try std.testing.expectApproxEqAbs(@as(f64, 1.0), x[0], 1e-9);
    try std.testing.expectApproxEqAbs(@as(f64, 1.0), x[1], 1e-9);
    try std.testing.expect(out.residual_norm <= 1e-12);
}

const Singular = struct {
    fn residual(self: *const Singular, x: []const f64, r: []f64) Validity {
        _ = self;
        r[0] = x[0] + x[1] - 1.0;
        r[1] = 2.0 * x[0] + 2.0 * x[1] - 2.0;
        return .valid;
    }
};

test "a singular Jacobian is reported without changing the iterate" {
    const system = Singular{};
    var x = [_]f64{ 0.0, 0.0 };
    var r: [2]f64 = undefined;
    const out = solve(Singular.residual, &system, &x, &r, .{});

    try std.testing.expectEqual(Status.singular, out.status);
    try std.testing.expect(out.pivot_ratio < 1e-12);
    try std.testing.expectEqual(@as(f64, 0.0), x[0]);
    try std.testing.expectEqual(@as(f64, -1.0), r[0]);
}

const Bounded = struct {
    fn residual(self: *const Bounded, x: []const f64, r: []f64) Validity {
        _ = self;
        if (x[0] < 0.0 or x[0] > 1.0) return .invalid;
        r[0] = x[0] - 0.5;
        return .valid;
    }
};

test "an out-of-bound unknown ends the solve" {
    const system = Bounded{};
    var x = [_]f64{2.0};
    var r: [1]f64 = undefined;
    const out = solve(Bounded.residual, &system, &x, &r, .{});

    try std.testing.expectEqual(Status.outside_envelope, out.status);
    try std.testing.expectEqual(@as(f64, 2.0), x[0]);
}

const NonFinite = struct {
    fn residual(self: *const NonFinite, x: []const f64, r: []f64) Validity {
        _ = self;
        r[0] = if (x[0] > 0.0) std.math.nan(f64) else -1.0;
        return .valid;
    }
};

test "a non-finite probe diverges without writing the iterate" {
    const system = NonFinite{};
    var x = [_]f64{0.0};
    var r: [1]f64 = undefined;
    const out = solve(NonFinite.residual, &system, &x, &r, .{});

    try std.testing.expectEqual(Status.diverged, out.status);
    try std.testing.expectEqual(@as(f64, 0.0), x[0]);
    try std.testing.expectEqual(@as(f64, -1.0), r[0]);
}

test "beyond_envelope is carried but does not stop convergence" {
    const system = Linear{ .validity = .beyond_envelope };
    var x = [_]f64{ 0.0, 0.0 };
    var r: [2]f64 = undefined;
    const out = solve(Linear.residual, &system, &x, &r, .{});

    try std.testing.expectEqual(Status.converged, out.status);
    try std.testing.expectEqual(Validity.beyond_envelope, out.validity);
    try std.testing.expectApproxEqAbs(@as(f64, 1.6), x[0], 1e-12);
}

const Far = struct {
    fn residual(self: *const Far, x: []const f64, r: []f64) Validity {
        _ = self;
        r[0] = x[0] - 100.0;
        return .valid;
    }
};

test "the correction obeys the step limit" {
    const system = Far{};
    var x = [_]f64{0.0};
    var r: [1]f64 = undefined;
    const options: Options = .{ .max_iterations = 3, .step_limit = 1.0 };
    const out = solve(Far.residual, &system, &x, &r, options);

    try std.testing.expectEqual(Status.max_iterations, out.status);
    try std.testing.expectApproxEqAbs(@as(f64, 3.0), x[0], 1e-12);
    try std.testing.expectEqual(@as(u16, 3), out.iterations);
}

test "the same problem gives bit-identical results twice" {
    const system = Linear{};
    var a = [_]f64{ 0.0, 0.0 };
    var ar: [2]f64 = undefined;
    var b = [_]f64{ 0.0, 0.0 };
    var br: [2]f64 = undefined;
    const first = solve(Linear.residual, &system, &a, &ar, .{});
    const second = solve(Linear.residual, &system, &b, &br, .{});

    try std.testing.expectEqualDeep(first, second);
    try std.testing.expectEqualDeep(a, b);
    try std.testing.expectEqualDeep(ar, br);
}

const UpperBoundRoot = struct {
    fn residual(self: *const UpperBoundRoot, x: []const f64, r: []f64) Validity {
        _ = self;
        if (x[0] < 0.0 or x[0] > 1.0) return .invalid;
        r[0] = x[0] - 1.0;
        return .valid;
    }
};

test "a root on its bound converges with a backward Jacobian probe" {
    const system = UpperBoundRoot{};
    var x = [_]f64{0.999999};
    var r: [1]f64 = undefined;
    const options: Options = .{ .tolerance = 1e-12 };
    const out = solve(UpperBoundRoot.residual, &system, &x, &r, options);

    try std.testing.expectEqual(Status.converged, out.status);
    try std.testing.expectApproxEqAbs(@as(f64, 1.0), x[0], 1e-12);
    try std.testing.expect(out.residual_norm <= 1e-12);
}
