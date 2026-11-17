//! Steady-flight trim of a compiled 6-DOF plan.
//!
//! A trim point is a state and a set of externals whose body velocity and rate derivatives
//! vanish. This first version solves the reduced, symmetric case: a level, wings-level
//! aircraft with an axial thruster, at a fixed speed, with `alpha` and `throttle` as the two
//! unknowns. `alpha` is a *binding* rather than an external: it writes the body velocity
//! slots `(V cos a, 0, V sin a)`, the attitude slots `fromEuler(0, a, 0)` and zero rate.
//! Controls remain externals. Side-force and moment derivatives are not solved; they are
//! checked to be zero by symmetry in the tests.
//!
//! Every evaluation copies the plan's initial state and the caller's baseline externals,
//! applies the bindings, calls `plan.rhs` once, and returns only the selected residual slots.
//! Scratch storage is caller-provided; the analysis allocates nothing.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const model = @import("model");
const solve = @import("solve");

const Plan = model.Plan;
const Workspace = model.Workspace;
const Validity = base.Validity;
const newton = solve.newton;
const limits = base.limits;

pub const Error = error{BadProblem};

/// Which model input one unknown drives.
pub const Binding = struct {
    pub const Kind = enum { external, alpha };

    kind: Kind = .external,
    /// External slot, used by `.external`.
    slot: u16 = 0,
    lower: f64 = -std.math.inf(f64),
    upper: f64 = std.math.inf(f64),
};

/// Offsets of the body state ports the flight condition writes.
pub const StateMap = struct {
    velocity: u16,
    rate: u16,
    attitude: u16,

    pub fn fromPlan(plan: *const Plan) Error!StateMap {
        return .{
            .velocity = plan.stateIndex("body.velocity_body") orelse return error.BadProblem,
            .rate = plan.stateIndex("body.rate_body") orelse return error.BadProblem,
            .attitude = plan.stateIndex("body.attitude") orelse return error.BadProblem,
        };
    }
};

pub const Problem = struct {
    plan: *const Plan,
    ws: *Workspace,
    /// Scratch state, externals and derivatives, all caller-owned.
    state: []f64,
    externals: []f64,
    derivatives: []f64,
    /// Baseline external values, also caller-owned; copied before each evaluation.
    baseline: []const f64,
    body: StateMap,
    speed_ms: f64,

    n: usize = 0,
    bindings: [limits.max_unknowns]Binding = @splat(.{}),
    residual_slots: [limits.max_unknowns]u16 = @splat(0),

    pub fn init(
        plan: *const Plan,
        ws: *Workspace,
        state: []f64,
        externals: []f64,
        derivatives: []f64,
        baseline: []const f64,
        speed_ms: f64,
    ) Error!Problem {
        if (state.len != plan.n_states or derivatives.len != plan.n_states) {
            return error.BadProblem;
        }
        if (externals.len != plan.n_external_slots or baseline.len != externals.len) {
            return error.BadProblem;
        }
        if (!(speed_ms > 0.0) or !std.math.isFinite(speed_ms)) return error.BadProblem;
        @memcpy(state, plan.initial_state);
        @memcpy(externals, baseline);
        return .{
            .plan = plan,
            .ws = ws,
            .state = state,
            .externals = externals,
            .derivatives = derivatives,
            .baseline = baseline,
            .body = try StateMap.fromPlan(plan),
            .speed_ms = speed_ms,
        };
    }

    /// The v1 problem: alpha and one control external, against x and z force derivatives.
    pub fn configureLevelFlight(self: *Problem, control_slot: u16, alpha_limit: f64) void {
        self.n = 2;
        self.bindings[0] = .{ .kind = .alpha, .lower = -alpha_limit, .upper = alpha_limit };
        self.bindings[1] = .{
            .kind = .external,
            .slot = control_slot,
            .lower = 0.0,
            .upper = 1.0,
        };
        self.residual_slots[0] = self.body.velocity;
        self.residual_slots[1] = self.body.velocity + 2;
    }

    /// Newton residual. Out-of-bound or non-finite unknowns are `invalid`; the plan's own
    /// validity (including `beyond_envelope`) is carried through.
    pub fn residual(self: *Problem, x: []const f64, r: []f64) Validity {
        assert(x.len == self.n and r.len == self.n);
        for (0..self.n) |i| {
            const bound = self.bindings[i];
            if (!std.math.isFinite(x[i]) or x[i] < bound.lower or x[i] > bound.upper) {
                return .invalid;
            }
        }

        @memcpy(self.state, self.plan.initial_state);
        @memcpy(self.externals, self.baseline);
        for (0..self.n) |i| {
            if (self.bindings[i].kind == .alpha) {
                self.applyAlpha(x[i]);
            } else {
                self.externals[self.bindings[i].slot] = x[i];
            }
        }

        const validity = self.plan.rhs(self.state, self.externals, self.ws, self.derivatives);
        for (0..self.n) |i| r[i] = self.derivatives[self.residual_slots[i]];
        return validity;
    }

    /// Runs the trim. `x0` has one entry per binding, in binding order.
    pub fn solve(self: *Problem, x0: []const f64, options: newton.Options) Result {
        assert(x0.len == self.n);
        var x: [limits.max_unknowns]f64 = undefined;
        var r: [limits.max_unknowns]f64 = undefined;
        @memcpy(x[0..self.n], x0);
        const outcome = newton.solve(Problem.residual, self, x[0..self.n], r[0..self.n], options);
        return .{
            .outcome = outcome,
            .unknowns = x,
            .state = self.state,
            .externals = self.externals,
            .derivatives = self.derivatives,
        };
    }

    fn applyAlpha(self: *Problem, alpha: f64) void {
        const v = self.body.velocity;
        self.state[v] = self.speed_ms * @cos(alpha);
        self.state[v + 1] = 0.0;
        self.state[v + 2] = self.speed_ms * @sin(alpha);

        const q = base.Quat.fromEuler(0.0, alpha, 0.0);
        const a = self.body.attitude;
        self.state[a] = q.w;
        self.state[a + 1] = q.x;
        self.state[a + 2] = q.y;
        self.state[a + 3] = q.z;
        @memset(self.state[self.body.rate..][0..3], 0.0);
    }
};

pub const Result = struct {
    outcome: newton.Outcome,
    /// Only the first `Problem.n` entries are meaningful.
    unknowns: [limits.max_unknowns]f64,
    /// Views of the caller's scratch; valid until the next residual evaluation.
    state: []const f64,
    externals: []const f64,
    derivatives: []const f64,
};

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

const aircraft = model.scenarios.trim_aircraft;
const test_plan_bytes = 256 * 1024;
const test_ws_bytes = 8 * 1024;

const Fixture = struct {
    plan_storage: [test_plan_bytes]u8 align(model.storage_alignment) = undefined,
    ws_storage: [test_ws_bytes]u8 align(model.storage_alignment) = undefined,
    state: [32]f64 = undefined,
    externals: [4]f64 = undefined,
    derivatives: [32]f64 = undefined,
    baseline: [2]f64 = .{ 0.0, 0.5 },
    diag: model.Diagnostic = .{},

    fn compile(self: *Fixture) !*const Plan {
        const comps = aircraft.components();
        const spec = aircraft.spec(&comps);
        return try model.compileInto(&self.plan_storage, &spec, &self.diag);
    }

    fn makeProblem(self: *Fixture, plan: *const Plan, alpha_limit: f64) !Problem {
        const ws = Workspace.into(plan, &self.ws_storage);
        var problem = try Problem.init(
            plan,
            ws,
            self.state[0..plan.n_states],
            self.externals[0..plan.n_external_slots],
            self.derivatives[0..plan.n_states],
            self.baseline[0..plan.n_external_slots],
            aircraft.speed_ms,
        );
        problem.configureLevelFlight(plan.signalIndex("throttle").?, alpha_limit);
        return problem;
    }
};

test "a level-flight trim balances lift and thrust" {
    var fixture = Fixture{};
    const plan = try fixture.compile();
    var problem = try fixture.makeProblem(plan, aircraft.alpha_stall_rad);
    const options: newton.Options = .{ .tolerance = 1e-11 };
    const result = problem.solve(&.{ 0.0, 0.5 }, options);

    try std.testing.expectEqual(newton.Status.converged, result.outcome.status);
    try std.testing.expectEqual(Validity.valid, result.outcome.validity);
    try std.testing.expect(result.outcome.residual_norm <= 1e-11);
    try std.testing.expect(result.outcome.iterations < 20);

    const alpha = result.unknowns[0];
    const throttle = result.unknowns[1];
    try std.testing.expectApproxEqAbs(@as(f64, 0.165), alpha, 0.01);
    try std.testing.expectApproxEqAbs(@as(f64, 0.82), throttle, 0.02);
    try std.testing.expect(@abs(alpha) < aircraft.alpha_stall_rad);

    const f = aircraft.forces(alpha, throttle);
    try std.testing.expectApproxEqRel(
        aircraft.mass_kg * aircraft.gravity_m_s2 * @cos(alpha),
        2.0 * f.lift,
        1e-9,
    );
    try std.testing.expectApproxEqRel(
        2.0 * f.drag + aircraft.mass_kg * aircraft.gravity_m_s2 * @sin(alpha),
        f.thrust,
        1e-9,
    );

    const velocity = plan.stateIndex("body.velocity_body").?;
    const rate = plan.stateIndex("body.rate_body").?;
    const attitude = plan.stateIndex("body.attitude").?;
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), result.state[velocity + 1], 1e-12);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), result.derivatives[velocity + 1], 1e-12);
    for (result.derivatives[rate..][0..3]) |d| {
        try std.testing.expectApproxEqAbs(@as(f64, 0.0), d, 1e-12);
    }
    const q = base.Quat{
        .w = result.state[attitude],
        .x = result.state[attitude + 1],
        .y = result.state[attitude + 2],
        .z = result.state[attitude + 3],
    };
    const world_v = q.rotate(.{
        .x = result.state[velocity],
        .y = result.state[velocity + 1],
        .z = result.state[velocity + 2],
    });
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), world_v.z, 1e-12);
    try std.testing.expectApproxEqAbs(aircraft.speed_ms, world_v.norm(), 1e-12);
}

test "a trim is deterministic and survives bad probes" {
    var fixture = Fixture{};
    const plan = try fixture.compile();
    var problem = try fixture.makeProblem(plan, aircraft.alpha_stall_rad);
    const options: newton.Options = .{ .tolerance = 1e-11 };
    const first = problem.solve(&.{ 0.0, 0.5 }, options);
    const second = problem.solve(&.{ 0.0, 0.5 }, options);
    try std.testing.expectEqualDeep(first.outcome, second.outcome);
    try std.testing.expectEqual(first.unknowns[0], second.unknowns[0]);
    try std.testing.expectEqual(first.unknowns[1], second.unknowns[1]);

    const nan = problem.solve(&.{ std.math.nan(f64), 0.5 }, options);
    try std.testing.expectEqual(newton.Status.diverged, nan.outcome.status);
    const inf = problem.solve(&.{ 0.0, std.math.inf(f64) }, options);
    try std.testing.expectEqual(newton.Status.diverged, inf.outcome.status);
    const out_of_bounds = problem.solve(&.{ 0.0, 1.5 }, options);
    try std.testing.expectEqual(
        newton.Status.outside_envelope,
        out_of_bounds.outcome.status,
    );
    for (problem.state) |value| try std.testing.expect(std.math.isFinite(value));
}

test "a stalled initial probe is beyond_envelope but can still converge" {
    var fixture = Fixture{};
    const plan = try fixture.compile();
    var problem = try fixture.makeProblem(plan, 0.5);
    const options: newton.Options = .{ .tolerance = 1e-10 };
    const result = problem.solve(&.{ 0.4, 0.5 }, options);

    try std.testing.expectEqual(newton.Status.converged, result.outcome.status);
    try std.testing.expectEqual(Validity.beyond_envelope, result.outcome.validity);
    try std.testing.expectApproxEqAbs(@as(f64, 0.165), result.unknowns[0], 0.01);
}
