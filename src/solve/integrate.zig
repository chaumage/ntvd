//! Fixed-step explicit time integration of a compiled plan.
//!
//! Classic fourth-order Runge-Kutta. Externals are held constant over a step (zero-order
//! hold); the caller updates them between steps to follow a schedule or a control law.
//!
//! The integrator is the guardian of the numerical-health invariant: it never hands a
//! non-finite state back. A step that would produce one fails with `error.Diverged` and
//! leaves the state untouched, so the caller keeps the last good point to report.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const model = @import("model");

const limits = base.limits;
const Plan = model.Plan;
const Workspace = model.Workspace;
const Validity = base.Validity;

pub const StepError = error{
    /// The step produced a non-finite state (or `dt` made no sense). State unchanged.
    Diverged,
};

/// Advances `x` by one step of length `dt` seconds. Returns the worst validity met in any
/// of the four stage evaluations.
pub fn rk4Step(
    plan: *const Plan,
    x: []f64,
    externals: []const f64,
    dt: f64,
    ws: *Workspace,
) StepError!Validity {
    const n = plan.n_states;
    assert(x.len == n and externals.len == plan.n_externals);
    if (!(dt > 0.0) or !std.math.isFinite(dt)) return error.Diverged;

    var k1: [limits.max_states]f64 = undefined;
    var k2: [limits.max_states]f64 = undefined;
    var k3: [limits.max_states]f64 = undefined;
    var k4: [limits.max_states]f64 = undefined;
    var stage: [limits.max_states]f64 = undefined;

    var worst = plan.rhs(x, externals, ws, k1[0..n]);
    axpy(stage[0..n], x, 0.5 * dt, k1[0..n]);
    worst = worst.worst(plan.rhs(stage[0..n], externals, ws, k2[0..n]));
    axpy(stage[0..n], x, 0.5 * dt, k2[0..n]);
    worst = worst.worst(plan.rhs(stage[0..n], externals, ws, k3[0..n]));
    axpy(stage[0..n], x, dt, k3[0..n]);
    worst = worst.worst(plan.rhs(stage[0..n], externals, ws, k4[0..n]));

    for (stage[0..n], x, k1[0..n], k2[0..n], k3[0..n], k4[0..n]) |*out, xi, a, b, c, d| {
        out.* = xi + dt / 6.0 * (a + 2.0 * b + 2.0 * c + d);
        if (!std.math.isFinite(out.*)) return error.Diverged;
    }
    @memcpy(x, stage[0..n]);
    return worst;
}

/// out = x + h * k
fn axpy(out: []f64, x: []const f64, h: f64, k: []const f64) void {
    for (out, x, k) |*o, xi, ki| o.* = xi + h * ki;
}

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

const scenarios = model.scenarios;

test "falling body converges to its analytic terminal velocity" {
    var plan: Plan = undefined;
    const comps = scenarios.falling_body.components(scenarios.falling_body.drag_surface());
    const spec = scenarios.falling_body.spec(&comps);
    var diag: model.Diagnostic = .{};
    try model.compile(&spec, &plan, &diag);

    const m = scenarios.falling_body.mass_kg;
    const g = base.units.g0;
    const vt = scenarios.falling_body.terminalVelocity();
    const ext = [_]f64{ 0.0, 0.0, m * g };

    var x = plan.initial_state;
    var ws: Workspace = .{};
    const dt = 0.01;
    const v_idx = plan.stateIndex("body.velocity").?;
    const p_idx = plan.stateIndex("body.position").?;

    // Exact solution of m dv/dt = m g - k v^2:  v = vt tanh(g t / vt),
    // position = vt^2 / g * ln cosh(g t / vt).
    var t: f64 = 0.0;
    for (0..1000) |step| {
        const validity = try rk4Step(&plan, x[0..plan.n_states], &ext, dt, &ws);
        try std.testing.expectEqual(Validity.valid, validity);
        t += dt;
        if (step % 100 == 99) {
            const v_exact = vt * std.math.tanh(g * t / vt);
            const p_exact = vt * vt / g * @log(std.math.cosh(g * t / vt));
            try std.testing.expectApproxEqAbs(v_exact, x[v_idx], 1e-8);
            try std.testing.expectApproxEqAbs(p_exact, x[p_idx], 1e-7);
        }
    }
    try std.testing.expectApproxEqRel(vt, x[v_idx], 1e-9);
}

test "rk4 is fourth-order accurate" {
    var plan: Plan = undefined;
    const comps = scenarios.falling_body.components(scenarios.falling_body.drag_surface());
    const spec = scenarios.falling_body.spec(&comps);
    var diag: model.Diagnostic = .{};
    try model.compile(&spec, &plan, &diag);

    const g = base.units.g0;
    const vt = scenarios.falling_body.terminalVelocity();
    const ext = [_]f64{ 0.0, 0.0, scenarios.falling_body.mass_kg * g };
    const v_idx = plan.stateIndex("body.velocity").?;
    const t_end = 1.0;
    const v_exact = vt * std.math.tanh(g * t_end / vt);

    var errors: [2]f64 = undefined;
    for ([_]u32{ 10, 20 }, 0..) |steps, i| {
        var x = plan.initial_state;
        var ws: Workspace = .{};
        const dt = t_end / @as(f64, @floatFromInt(steps));
        for (0..steps) |_| _ = try rk4Step(&plan, x[0..plan.n_states], &ext, dt, &ws);
        errors[i] = @abs(x[v_idx] - v_exact);
    }
    // Halving the step divides the error by 2^4 = 16 (observed order between 3.5 and 4.5).
    const order = std.math.log2(errors[0] / errors[1]);
    try std.testing.expect(order > 3.5 and order < 4.5);
}

test "a diverging simulation is reported and the last good state is kept" {
    var plan: Plan = undefined;
    const comps = scenarios.falling_body.components(scenarios.falling_body.drag_surface());
    const spec = scenarios.falling_body.spec(&comps);
    var diag: model.Diagnostic = .{};
    try model.compile(&spec, &plan, &diag);

    // A force of 1e307 N on 2 kg overflows the velocity within two steps.
    const ext = [_]f64{ 0.0, 0.0, 1.0e307 };
    var x = plan.initial_state;
    var ws: Workspace = .{};
    var last_good = x;
    var diverged = false;
    for (0..10) |_| {
        last_good = x;
        _ = rk4Step(&plan, x[0..plan.n_states], &ext, 1.0e3, &ws) catch |err| {
            try std.testing.expectEqual(error.Diverged, err);
            diverged = true;
            break;
        };
    }
    try std.testing.expect(diverged);
    try std.testing.expectEqualSlices(f64, last_good[0..plan.n_states], x[0..plan.n_states]);
    for (x[0..2]) |v| try std.testing.expect(std.math.isFinite(v));
}

test "nonsense time steps are refused" {
    var plan: Plan = undefined;
    const comps = scenarios.falling_body.components(scenarios.falling_body.drag_surface());
    const spec = scenarios.falling_body.spec(&comps);
    var diag: model.Diagnostic = .{};
    try model.compile(&spec, &plan, &diag);
    var x = plan.initial_state;
    var ws: Workspace = .{};
    const ext = [_]f64{ 0.0, 0.0, 1.0 };
    for ([_]f64{ 0.0, -1.0, std.math.nan(f64), std.math.inf(f64) }) |dt| {
        const result = rk4Step(&plan, x[0..plan.n_states], &ext, dt, &ws);
        try std.testing.expectError(error.Diverged, result);
    }
}
