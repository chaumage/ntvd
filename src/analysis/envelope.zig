//! Directional envelope of a trimmed aircraft.
//!
//! For a powered plan this sweeps airspeed, trims alpha and the control at each speed, brackets
//! the last feasible point, refines locally and verifies it with a final trim. For a glider the
//! trim itself solves alpha and speed, so the envelope is that single equilibrium. The sweep
//! never assumes global monotonicity: the coarse scan finds the last feasible island first.
//!
//! Directions are unit vectors in world NED (`+z` is down). A limiting classification is made
//! by checking the final trim's bounds; `beyond_envelope` is reported, never treated as a
//! failure. Scratch belongs to the caller's `trim.Problem`; nothing here allocates.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const model = @import("model");
const trim = @import("trim.zig");
const solve_layer = @import("solve");

const newton = solve_layer.newton;

/// Why the maximum was reached, or why no trim was feasible.
pub const Limit = enum { none, thrust, lift, speed_bounds, validity };

pub const Options = struct {
    v_min: f64 = 5.0,
    v_max: f64 = 120.0,
    /// Coarse scan intervals over `[v_min, v_max]`.
    scan_steps: u16 = 32,
    /// Local feasibility refinements inside the last bracket.
    refine_steps: u16 = 24,
    newton: newton.Options = .{},
};

pub const Point = struct {
    direction: base.Vec3,
    /// Zero when no trim was feasible.
    v_max: f64,
    limiting: Limit,
    feasible: bool,
    /// Scratch views; valid until the next trim evaluation.
    trim: trim.Result,
};

/// Fills `dest` with one envelope point per direction, in caller order. Returns the count.
pub fn sweep(
    problem: *trim.Problem,
    directions: []const base.Vec3,
    dest: []Point,
    opts: Options,
) usize {
    assert(dest.len >= directions.len);
    for (directions, 0..) |direction, i| dest[i] = solve(problem, direction, opts);
    return directions.len;
}

/// Computes the maximum feasible speed along `direction`.
pub fn solve(problem: *trim.Problem, direction: base.Vec3, opts: Options) Point {
    assert(opts.v_min > 0.0 and opts.v_max > opts.v_min);
    assert(opts.scan_steps >= 1 and opts.refine_steps >= 1);
    problem.setDirection(direction);
    if (problem.mode == .glider) return solveGlider(problem, direction, opts);
    return solvePowered(problem, direction, opts);
}

/// A single glider equilibrium: alpha and speed are both trim unknowns.
fn solveGlider(problem: *trim.Problem, direction: base.Vec3, opts: Options) Point {
    const bounds = problem.speedBounds();
    const initial_speed = 0.5 * (bounds.lower + bounds.upper);
    const result = problem.solve(&.{ 0.0, initial_speed }, opts.newton);
    const speed = if (isFeasible(result)) problem.speed_ms else 0.0;
    return makePoint(problem, direction, speed, result, opts);
}

/// Sweeps speed, brackets the last feasible trim and refines the boundary.
fn solvePowered(problem: *trim.Problem, direction: base.Vec3, opts: Options) Point {
    var alpha: f64 = 0.0;
    var control: f64 = 0.5;
    var lo = opts.v_min;
    var hi = opts.v_max;
    var best = trimAt(problem, lo, alpha, control, opts);
    remember(problem, best, &alpha, &control);

    if (!isFeasible(best)) {
        return makePoint(problem, direction, 0.0, best, opts);
    }

    const steps: f64 = @floatFromInt(opts.scan_steps);
    for (1..opts.scan_steps + 1) |k| {
        const v = opts.v_min + (opts.v_max - opts.v_min) * @as(f64, @floatFromInt(k)) / steps;
        const result = trimAt(problem, v, alpha, control, opts);
        remember(problem, result, &alpha, &control);
        if (isFeasible(result)) {
            lo = v;
            best = result;
        } else {
            hi = v;
            break;
        }
    }

    for (0..opts.refine_steps) |_| {
        const mid = 0.5 * (lo + hi);
        const result = trimAt(problem, mid, alpha, control, opts);
        remember(problem, result, &alpha, &control);
        if (isFeasible(result)) {
            lo = mid;
            best = result;
        } else {
            hi = mid;
        }
    }

    const verified = trimAt(problem, lo, alpha, control, opts);
    return makePoint(problem, direction, lo, verified, opts);
}

fn trimAt(problem: *trim.Problem, speed: f64, alpha: f64, control: f64, opts: Options) trim.Result {
    problem.setSpeed(speed);
    return problem.solve(&.{ alpha, control }, opts.newton);
}

fn remember(problem: *const trim.Problem, result: trim.Result, alpha: *f64, control: *f64) void {
    if (std.math.isFinite(result.unknowns[0])) alpha.* = result.unknowns[0];
    const guess = result.unknowns[problem.control_index];
    if (std.math.isFinite(guess)) control.* = guess;
}

fn isFeasible(result: trim.Result) bool {
    return result.outcome.status == .converged and result.outcome.validity != .invalid;
}

fn makePoint(
    problem: *const trim.Problem,
    direction: base.Vec3,
    speed: f64,
    result: trim.Result,
    opts: Options,
) Point {
    return .{
        .direction = unit(direction),
        .v_max = speed,
        .limiting = classify(problem, result, speed, opts),
        .feasible = isFeasible(result),
        .trim = result,
    };
}

fn classify(problem: *const trim.Problem, result: trim.Result, speed: f64, opts: Options) Limit {
    const control_tol = 1.0e-6;
    const alpha_limit = problem.bindings[0].upper;
    const alpha_tol = 1.0e-3 * @max(1.0, alpha_limit);
    if (!isFeasible(result)) {
        if (alphaAtBound(problem, result, alpha_tol)) return .lift;
        if (problem.mode == .powered and result.outcome.status == .outside_envelope) return .thrust;
        return .validity;
    }
    if (alphaAtBound(problem, result, alpha_tol)) return .lift;
    if (problem.mode == .glider) {
        if (speed >= problem.speedBounds().upper - control_tol) return .speed_bounds;
    } else {
        const control = result.unknowns[problem.control_index];
        if (control >= 1.0 - control_tol) return .thrust;
    }
    if (speed >= opts.v_max - control_tol) return .speed_bounds;
    return .none;
}

fn alphaAtBound(problem: *const trim.Problem, result: trim.Result, tol: f64) bool {
    const upper = problem.bindings[0].upper;
    return @abs(result.unknowns[0]) >= upper - tol;
}

fn unit(direction: base.Vec3) base.Vec3 {
    assert(direction.isFinite() and direction.norm() > 0.0);
    return direction.normalized().?;
}

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

const powered = model.scenarios.trim_aircraft;
const glider = model.scenarios.glider;
const test_plan_bytes = 256 * 1024;
const test_ws_bytes = 8 * 1024;

const Fixture = struct {
    plan_storage: [test_plan_bytes]u8 align(model.storage_alignment) = undefined,
    ws_storage: [test_ws_bytes]u8 align(model.storage_alignment) = undefined,
    state: [32]f64 = undefined,
    externals: [4]f64 = undefined,
    derivatives: [32]f64 = undefined,
    powered_baseline: [2]f64 = .{ 0.0, 0.5 },
    glider_baseline: [1]f64 = .{0.0},
    diag: model.Diagnostic = .{},

    fn compile(self: *Fixture, spec: *const model.Spec) !*const model.Plan {
        return try model.compileInto(&self.plan_storage, spec, &self.diag);
    }

    fn workspace(self: *Fixture, plan: *const model.Plan) *model.Workspace {
        return model.Workspace.into(plan, &self.ws_storage);
    }

    fn makePowered(
        self: *Fixture,
        plan: *const model.Plan,
        direction: base.Vec3,
        alpha_limit: f64,
    ) !trim.Problem {
        var problem = try trim.Problem.init(
            plan,
            self.workspace(plan),
            self.state[0..plan.n_states],
            self.externals[0..plan.n_external_slots],
            self.derivatives[0..plan.n_states],
            self.powered_baseline[0..plan.n_external_slots],
            powered.speed_ms,
        );
        problem.configureDirection(direction, plan.signalIndex("throttle").?, alpha_limit);
        return problem;
    }

    fn makeGlider(self: *Fixture, plan: *const model.Plan, direction: base.Vec3) !trim.Problem {
        var problem = try trim.Problem.init(
            plan,
            self.workspace(plan),
            self.state[0..plan.n_states],
            self.externals[0..plan.n_external_slots],
            self.derivatives[0..plan.n_states],
            self.glider_baseline[0..plan.n_external_slots],
            20.0,
        );
        problem.configureGlide(direction, glider.alpha_stall_rad, 10.0, 120.0);
        return problem;
    }
};

const envelope_options: Options = .{ .newton = .{ .tolerance = 1e-10 } };

fn poweredPlan(fixture: *Fixture) !*const model.Plan {
    const comps = powered.components();
    return try fixture.compile(&powered.spec(&comps));
}

fn gliderPlan(fixture: *Fixture) !*const model.Plan {
    const comps = glider.components();
    return try fixture.compile(&glider.spec(&comps));
}

fn velocityOf(plan: *const model.Plan, result: trim.Result) base.Vec3 {
    const v = plan.stateIndex("body.velocity_body").?;
    return .{ .x = result.state[v], .y = result.state[v + 1], .z = result.state[v + 2] };
}

fn attitudeOf(plan: *const model.Plan, result: trim.Result) base.Quat {
    const a = plan.stateIndex("body.attitude").?;
    return .{
        .w = result.state[a],
        .x = result.state[a + 1],
        .y = result.state[a + 2],
        .z = result.state[a + 3],
    };
}

test "a glider's vertical dive matches the analytic terminal velocity" {
    var fixture = Fixture{};
    const plan = try gliderPlan(&fixture);
    const down = base.Vec3{ .x = 0.0, .y = 0.0, .z = 1.0 };
    var problem = try fixture.makeGlider(plan, down);
    const point = solve(&problem, down, envelope_options);
    const alpha = point.trim.unknowns[0];

    try std.testing.expect(point.feasible);
    try std.testing.expectEqual(newton.Status.converged, point.trim.outcome.status);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), alpha, 1e-8);
    try std.testing.expectApproxEqRel(glider.terminalSpeed(), point.v_max, 1e-6);
    try std.testing.expectEqual(Limit.none, point.limiting);

    const forces = glider.forces(alpha, point.v_max);
    try std.testing.expectApproxEqRel(glider.mass_kg * glider.gravity_m_s2, forces.drag, 1e-8);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), forces.lift, 1e-8);

    const world = attitudeOf(plan, point.trim).rotate(velocityOf(plan, point.trim));
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), world.x, 1e-10);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), world.y, 1e-10);
    try std.testing.expectApproxEqRel(1.0, world.z / point.v_max, 1e-10);
}

test "a powered aircraft's maximum is thrust-limited" {
    var fixture = Fixture{};
    const plan = try poweredPlan(&fixture);
    const forward = base.Vec3{ .x = 1.0, .y = 0.0, .z = 0.0 };
    var problem = try fixture.makePowered(plan, forward, powered.alpha_stall_rad);
    const opts: Options = .{ .v_min = 20.0, .v_max = 59.0, .newton = .{ .tolerance = 1e-10 } };
    const point = solve(&problem, forward, opts);
    const alpha = point.trim.unknowns[0];
    const throttle = point.trim.unknowns[problem.control_index];

    try std.testing.expect(point.feasible);
    try std.testing.expectEqual(Limit.thrust, point.limiting);
    try std.testing.expectApproxEqAbs(@as(f64, 1.0), throttle, 1e-7);
    try std.testing.expect(point.v_max > 35.0 and point.v_max < 59.0);

    const drag = 2.0 * powered.wingDrag(alpha, point.v_max);
    const required = drag + powered.mass_kg * powered.gravity_m_s2 * @sin(alpha);
    const available = powered.thrust(throttle, point.v_max);
    try std.testing.expectApproxEqRel(required, available, 1e-8);
}

test "an aircraft that stalls before cruising reports a lift limit" {
    var fixture = Fixture{};
    const plan = try poweredPlan(&fixture);
    const forward = base.Vec3{ .x = 1.0, .y = 0.0, .z = 0.0 };
    var problem = try fixture.makePowered(plan, forward, 0.02);
    const opts: Options = .{ .v_min = 20.0, .v_max = 59.0, .newton = .{ .tolerance = 1e-10 } };
    const point = solve(&problem, forward, opts);

    try std.testing.expect(!point.feasible);
    try std.testing.expectEqual(Limit.lift, point.limiting);
    try std.testing.expectEqual(@as(f64, 0.0), point.v_max);
}

test "a sweep is symmetric, ordered and deterministic" {
    var fixture = Fixture{};
    const plan = try poweredPlan(&fixture);
    const directions = [_]base.Vec3{
        .{ .x = 1.0, .y = 0.0, .z = 0.0 },
        .{ .x = 0.0, .y = 1.0, .z = 0.0 },
        .{ .x = 0.0, .y = -1.0, .z = 0.0 },
    };
    var first: [directions.len]Point = undefined;
    var second: [directions.len]Point = undefined;
    const opts: Options = .{ .v_min = 20.0, .v_max = 59.0, .newton = .{ .tolerance = 1e-10 } };
    var problem = try fixture.makePowered(plan, directions[0], powered.alpha_stall_rad);
    try std.testing.expectEqual(directions.len, sweep(&problem, &directions, &first, opts));
    try std.testing.expectEqual(directions.len, sweep(&problem, &directions, &second, opts));

    for (first, second) |a, b| {
        try std.testing.expectEqualDeep(a.trim.outcome, b.trim.outcome);
        try std.testing.expectEqual(a.v_max, b.v_max);
        try std.testing.expectEqual(a.limiting, b.limiting);
    }
    try std.testing.expectEqual(first[1].v_max, first[2].v_max);
    try std.testing.expectEqual(first[1].limiting, first[2].limiting);
    try std.testing.expectEqual(@as(f64, 1.0), first[1].direction.y);
    try std.testing.expectEqual(@as(f64, -1.0), first[2].direction.y);
}
