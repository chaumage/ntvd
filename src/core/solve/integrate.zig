//! Fixed-step explicit time integration of a compiled plan.
//!
//! Classic fourth-order Runge-Kutta. Externals are held constant over a step (zero-order
//! hold); the caller updates them between steps to follow a schedule or a control law.
//!
//! The integrator is the guardian of the numerical-health invariant: it never hands a
//! non-finite state back. A step that would produce one fails with `error.Diverged` and
//! leaves the state untouched, so the caller keeps the last good point to report.
//!
//! Storage follows the model layer: `Workspace` is a header carved from caller-provided
//! storage, holding the model workspace plus the RK4 stage scratch.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const model = @import("model");
const storage = model.storage;

const Plan = model.Plan;
const Validity = base.Validity;

/// Scratch of one integration: the model workspace and five state vectors (four stages plus
/// the trial state).
pub const Workspace = struct {
    model: model.Workspace,
    scratch: []f64,

    /// Bytes needed for `plan`, including this header.
    pub fn bytesNeeded(plan: *const Plan) usize {
        var sizer: storage.Sizer = .{};
        sizer.add(Workspace, 1);
        addTo(plan, &sizer);
        return sizer.bytes();
    }

    /// Contributes the sizes of the arrays, for a workspace embedded in a larger struct.
    pub fn addTo(plan: *const Plan, sizer: *storage.Sizer) void {
        model.Workspace.addTo(plan, sizer);
        sizer.add(f64, 5 * plan.n_states);
    }

    /// Builds a workspace in `buffer`, which must be at least `bytesNeeded(plan)` long.
    pub fn into(plan: *const Plan, buffer: storage.Buffer) *Workspace {
        var carver = storage.Carver.init(buffer);
        const ws = carver.takeOne(Workspace);
        ws.* = .{
            .model = model.Workspace.carve(plan, &carver),
            .scratch = carver.take(f64, 5 * plan.n_states),
        };
        assert(carver.cursor == bytesNeeded(plan));
        return ws;
    }
};

pub const storage_alignment = storage.alignment;

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
    const n: usize = plan.n_states;
    assert(x.len == n and externals.len == plan.n_external_slots);
    assert(ws.model.signals.len == plan.n_signals);
    assert(ws.scratch.len == 5 * n);
    if (!(dt > 0.0) or !std.math.isFinite(dt)) return error.Diverged;

    const k1 = ws.scratch[0..n];
    const k2 = ws.scratch[n..][0..n];
    const k3 = ws.scratch[2 * n ..][0..n];
    const k4 = ws.scratch[3 * n ..][0..n];
    const stage = ws.scratch[4 * n ..][0..n];

    var worst = plan.rhs(x, externals, &ws.model, k1);
    axpy(stage, x, 0.5 * dt, k1);
    worst = worst.worst(plan.rhs(stage, externals, &ws.model, k2));
    axpy(stage, x, 0.5 * dt, k2);
    worst = worst.worst(plan.rhs(stage, externals, &ws.model, k3));
    axpy(stage, x, dt, k3);
    worst = worst.worst(plan.rhs(stage, externals, &ws.model, k4));

    for (stage, x, k1, k2, k3, k4) |*out, xi, a, b, c, d| {
        out.* = xi + dt / 6.0 * (a + 2.0 * b + 2.0 * c + d);
        if (!std.math.isFinite(out.*)) return error.Diverged;
    }
    try normalizeAttitudes(plan, stage);
    @memcpy(x, stage);
    return worst;
}

/// out = x + h * k
fn axpy(out: []f64, x: []const f64, h: f64, k: []const f64) void {
    for (out, x, k) |*o, xi, ki| o.* = xi + h * ki;
}

/// Renormalizes every quaternion state in `state`. On a zero or non-finite norm the step is
/// refused before the caller's state is touched.
fn normalizeAttitudes(plan: *const Plan, state: []f64) StepError!void {
    for (plan.attitude_states) |offset| {
        const start: usize = offset;
        assert(start + 4 <= state.len);
        const values = state[start..][0..4];
        const q = base.Quat{
            .w = values[0],
            .x = values[1],
            .y = values[2],
            .z = values[3],
        };
        const normalized = base.Quat.normalized(q) orelse return error.Diverged;
        values[0] = normalized.w;
        values[1] = normalized.x;
        values[2] = normalized.y;
        values[3] = normalized.z;
    }
}

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

const scenarios = model.scenarios;

/// Comfortably larger than the falling-body plan; the exact size is `bytesNeeded(spec)`.
const test_plan_bytes = 256 * 1024;
const test_ws_bytes = 8 * 1024;

const falling = struct {
    const comps = scenarios.falling_body.components(scenarios.falling_body.drag_surface());
    const spec = scenarios.falling_body.spec(&comps);
    const n_states = 2;

    const Built = struct { plan: *const Plan, ws: *Workspace };

    /// The storage must outlive the returned pointers.
    fn build(
        plan_storage: storage.Buffer,
        ws_storage: storage.Buffer,
        diag: *model.Diagnostic,
    ) !Built {
        const plan = try model.compileInto(plan_storage, &spec, diag);
        return .{ .plan = plan, .ws = Workspace.into(plan, ws_storage) };
    }
};

test "falling body converges to its analytic terminal velocity" {
    var plan_storage: [test_plan_bytes]u8 align(storage.alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(storage.alignment) = undefined;
    var diag: model.Diagnostic = .{};
    const built = try falling.build(&plan_storage, &ws_storage, &diag);
    const plan = built.plan;

    const m = scenarios.falling_body.mass_kg;
    const g = base.units.g0;
    const vt = scenarios.falling_body.terminalVelocity();
    const ext = [_]f64{ 0.0, 0.0, m * g };

    var x: [falling.n_states]f64 = undefined;
    @memcpy(&x, plan.initial_state);
    const dt = 0.01;
    const v_idx = plan.stateIndex("body.velocity").?;
    const p_idx = plan.stateIndex("body.position").?;

    // Exact solution of m dv/dt = m g - k v^2:  v = vt tanh(g t / vt),
    // position = vt^2 / g * ln cosh(g t / vt).
    var t: f64 = 0.0;
    for (0..1000) |step| {
        const validity = try rk4Step(plan, &x, &ext, dt, built.ws);
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
    var plan_storage: [test_plan_bytes]u8 align(storage.alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(storage.alignment) = undefined;
    var diag: model.Diagnostic = .{};
    const built = try falling.build(&plan_storage, &ws_storage, &diag);
    const plan = built.plan;

    const g = base.units.g0;
    const vt = scenarios.falling_body.terminalVelocity();
    const ext = [_]f64{ 0.0, 0.0, scenarios.falling_body.mass_kg * g };
    const v_idx = plan.stateIndex("body.velocity").?;
    const t_end = 1.0;
    const v_exact = vt * std.math.tanh(g * t_end / vt);

    var errors: [2]f64 = undefined;
    for ([_]u32{ 10, 20 }, 0..) |steps, i| {
        var x: [falling.n_states]f64 = undefined;
        @memcpy(&x, plan.initial_state);
        const dt = t_end / @as(f64, @floatFromInt(steps));
        for (0..steps) |_| _ = try rk4Step(plan, &x, &ext, dt, built.ws);
        errors[i] = @abs(x[v_idx] - v_exact);
    }
    // Halving the step divides the error by 2^4 = 16 (observed order between 3.5 and 4.5).
    const order = std.math.log2(errors[0] / errors[1]);
    try std.testing.expect(order > 3.5 and order < 4.5);
}

test "a diverging simulation is reported and the last good state is kept" {
    var plan_storage: [test_plan_bytes]u8 align(storage.alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(storage.alignment) = undefined;
    var diag: model.Diagnostic = .{};
    const built = try falling.build(&plan_storage, &ws_storage, &diag);
    const plan = built.plan;

    // A force of 1e307 N on 2 kg overflows the velocity within two steps.
    const ext = [_]f64{ 0.0, 0.0, 1.0e307 };
    var x: [falling.n_states]f64 = undefined;
    @memcpy(&x, plan.initial_state);
    var last_good = x;
    var diverged = false;
    for (0..10) |_| {
        last_good = x;
        _ = rk4Step(plan, &x, &ext, 1.0e3, built.ws) catch |err| {
            try std.testing.expectEqual(error.Diverged, err);
            diverged = true;
            break;
        };
    }
    try std.testing.expect(diverged);
    try std.testing.expectEqualSlices(f64, &last_good, &x);
    for (x) |v| try std.testing.expect(std.math.isFinite(v));
}

test "nonsense time steps are refused" {
    var plan_storage: [test_plan_bytes]u8 align(storage.alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(storage.alignment) = undefined;
    var diag: model.Diagnostic = .{};
    const built = try falling.build(&plan_storage, &ws_storage, &diag);
    const plan = built.plan;
    var x: [falling.n_states]f64 = undefined;
    @memcpy(&x, plan.initial_state);
    const ext = [_]f64{ 0.0, 0.0, 1.0 };
    for ([_]f64{ 0.0, -1.0, std.math.nan(f64), std.math.inf(f64) }) |dt| {
        const result = rk4Step(plan, &x, &ext, dt, built.ws);
        try std.testing.expectError(error.Diverged, result);
    }
}

// ---------------------------------------------------------------------------------------
// Six-degree-of-freedom body
// ---------------------------------------------------------------------------------------

/// A body alone, driven by one body-frame wrench external. The mount port sums, so a zero
/// external means no applied wrench.
const six_dof = struct {
    const mass_kg = 2.0;
    const inertia: base.Vec3 = .{ .x = 1.0, .y = 2.0, .z = 4.0 };
    const n_states = 13;

    const externals = [_]model.ExternalSpec{.{
        .name = "w",
        .dim = base.dim.force,
        .shape = .wrench,
        .quantity = .wrench,
        .frame = .body,
    }};

    const connections = [_]model.ConnectionSpec{
        .{ .from = "w", .to = "body.mount" },
    };

    const Built = struct { plan: *const Plan, ws: *Workspace };

    fn build(
        plan_storage: storage.Buffer,
        ws_storage: storage.Buffer,
        diag: *model.Diagnostic,
        gravity_m_s2: f64,
    ) !Built {
        var comps = [_]model.ComponentSpec{.{
            .name = "body",
            .component = .{ .body = .{ .six_dof = .{
                .mass_kg = mass_kg,
                .inertia_kg_m2 = inertia,
                .gravity_m_s2 = gravity_m_s2,
            } } },
        }};
        const spec: model.Spec = .{
            .components = &comps,
            .externals = &externals,
            .connections = &connections,
        };
        const plan = try model.compileInto(plan_storage, &spec, diag);
        return .{ .plan = plan, .ws = Workspace.into(plan, ws_storage) };
    }
};

fn stateVec(plan: *const Plan, x: []const f64, name: []const u8) base.Vec3 {
    const i = plan.stateIndex(name).?;
    return .{ .x = x[i], .y = x[i + 1], .z = x[i + 2] };
}

fn stateQuat(plan: *const Plan, x: []const f64, name: []const u8) base.Quat {
    const i = plan.stateIndex(name).?;
    return .{ .w = x[i], .x = x[i + 1], .y = x[i + 2], .z = x[i + 3] };
}

fn writeQuat(x: []f64, index: u16, q: base.Quat) void {
    const i: usize = index;
    x[i] = q.w;
    x[i + 1] = q.x;
    x[i + 2] = q.y;
    x[i + 3] = q.z;
}

fn signalVec(ws: *const model.Workspace, plan: *const Plan, name: []const u8) base.Vec3 {
    const slots = plan.signalSlots(name).?;
    return .{
        .x = ws.signals[slots.index],
        .y = ws.signals[slots.index + 1],
        .z = ws.signals[slots.index + 2],
    };
}

fn expectVec(expected: base.Vec3, actual: base.Vec3) !void {
    try std.testing.expectApproxEqAbs(expected.x, actual.x, 1.0e-12);
    try std.testing.expectApproxEqAbs(expected.y, actual.y, 1.0e-12);
    try std.testing.expectApproxEqAbs(expected.z, actual.z, 1.0e-12);
}

fn normSquared(q: base.Quat) f64 {
    return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
}

test "a constant body force accelerates in body and world frames" {
    var plan_storage: [test_plan_bytes]u8 align(storage.alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(storage.alignment) = undefined;
    var diag: model.Diagnostic = .{};
    const built = try six_dof.build(&plan_storage, &ws_storage, &diag, 0.0);
    const plan = built.plan;

    var x: [six_dof.n_states]f64 = undefined;
    plan.copyInitialState(&x);
    // Yaw 90 degrees: a body +x force points along world +y.
    writeQuat(
        &x,
        plan.stateIndex("body.attitude").?,
        base.Quat.fromEuler(0.0, 0.0, std.math.pi / 2.0),
    );

    const ext = [_]f64{ 2.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    const dt = 0.01;
    for (0..100) |_| _ = try rk4Step(plan, &x, &ext, dt, built.ws);

    try expectVec(.{ .x = 1.0 }, stateVec(plan, &x, "body.velocity_body"));
    try expectVec(.{ .y = 0.5 }, stateVec(plan, &x, "body.position_world"));

    _ = plan.evaluate(&x, &ext, &built.ws.model);
    try expectVec(.{ .y = 1.0 }, signalVec(&built.ws.model, plan, "body.velocity_world"));
}

test "free fall in NED matches z = half g t squared" {
    var plan_storage: [test_plan_bytes]u8 align(storage.alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(storage.alignment) = undefined;
    var diag: model.Diagnostic = .{};
    const built = try six_dof.build(&plan_storage, &ws_storage, &diag, base.units.g0);
    const plan = built.plan;

    var x: [six_dof.n_states]f64 = undefined;
    plan.copyInitialState(&x);
    const ext = [_]f64{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    const dt = 1.0e-3;
    const steps = 10_000;
    for (0..steps) |_| _ = try rk4Step(plan, &x, &ext, dt, built.ws);

    const t = dt * @as(f64, @floatFromInt(steps));
    const z = 0.5 * base.units.g0 * t * t;
    try std.testing.expectApproxEqAbs(z, stateVec(plan, &x, "body.position_world").z, 1.0e-8);
    try std.testing.expectApproxEqAbs(
        base.units.g0 * t,
        stateVec(plan, &x, "body.velocity_body").z,
        1.0e-10,
    );
}

test "a torque-free spinner conserves angular momentum and energy" {
    var plan_storage: [test_plan_bytes]u8 align(storage.alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(storage.alignment) = undefined;
    var diag: model.Diagnostic = .{};
    const built = try six_dof.build(&plan_storage, &ws_storage, &diag, 0.0);
    const plan = built.plan;

    const rate: base.Vec3 = .{ .x = 1.0, .y = 2.0, .z = 3.0 };
    const h0 = base.Vec3{
        .x = six_dof.inertia.x * rate.x,
        .y = six_dof.inertia.y * rate.y,
        .z = six_dof.inertia.z * rate.z,
    };
    const energy0 = 0.5 * base.Vec3.dot(rate, h0);
    const ext = [_]f64{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    var x: [six_dof.n_states]f64 = undefined;
    plan.copyInitialState(&x);
    const rate_index = plan.stateIndex("body.rate_body").?;
    x[rate_index] = rate.x;
    x[rate_index + 1] = rate.y;
    x[rate_index + 2] = rate.z;

    const dt = 1.0e-3;
    for (0..10_000) |_| {
        _ = try rk4Step(plan, &x, &ext, dt, built.ws);
        try std.testing.expectApproxEqAbs(
            1.0,
            normSquared(stateQuat(plan, &x, "body.attitude")),
            1.0e-12,
        );
    }

    const q = stateQuat(plan, &x, "body.attitude");
    const omega = stateVec(plan, &x, "body.rate_body");
    const h = base.Vec3{
        .x = six_dof.inertia.x * omega.x,
        .y = six_dof.inertia.y * omega.y,
        .z = six_dof.inertia.z * omega.z,
    };
    const h_world = q.rotate(h);
    const energy = 0.5 * base.Vec3.dot(omega, h);
    try std.testing.expectApproxEqRel(h0.x, h_world.x, 1.0e-8);
    try std.testing.expectApproxEqRel(h0.y, h_world.y, 1.0e-8);
    try std.testing.expectApproxEqRel(h0.z, h_world.z, 1.0e-8);
    try std.testing.expectApproxEqRel(energy0, energy, 1.0e-8);
}

test "a zero quaternion stops the step and keeps the last good state" {
    var plan_storage: [test_plan_bytes]u8 align(storage.alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(storage.alignment) = undefined;
    var diag: model.Diagnostic = .{};
    const built = try six_dof.build(&plan_storage, &ws_storage, &diag, 0.0);
    const plan = built.plan;

    var x: [six_dof.n_states]f64 = undefined;
    plan.copyInitialState(&x);
    writeQuat(
        &x,
        plan.stateIndex("body.attitude").?,
        .{ .w = 0.0, .x = 0.0, .y = 0.0, .z = 0.0 },
    );
    const before = x;
    const ext = [_]f64{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    try std.testing.expectError(
        error.Diverged,
        rk4Step(plan, &x, &ext, 0.01, built.ws),
    );
    try std.testing.expectEqualSlices(f64, &before, &x);
}

test "the body stays readable when the mount signal is broken" {
    var plan_storage: [test_plan_bytes]u8 align(storage.alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(storage.alignment) = undefined;
    var diag: model.Diagnostic = .{};
    const built = try six_dof.build(&plan_storage, &ws_storage, &diag, 0.0);
    const plan = built.plan;

    var x: [six_dof.n_states]f64 = undefined;
    plan.copyInitialState(&x);
    const ext = [_]f64{ std.math.nan(f64), 0.0, 0.0, 0.0, 0.0, 0.0 };

    // The body is not feedthrough, so its outputs do not read the broken mount.
    try std.testing.expectEqual(Validity.valid, plan.evaluate(&x, &ext, &built.ws.model));

    var dx: [six_dof.n_states]f64 = undefined;
    try std.testing.expectEqual(
        Validity.invalid,
        plan.derivatives(&x, &built.ws.model, &dx),
    );
    for (dx) |value| try std.testing.expectEqual(@as(f64, 0.0), value);
}
