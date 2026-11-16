//! End-to-end architectures that exercise the model layer the way an engineer will.
//!
//! These are the tests that prove the abstractions hold together: a physical feedback loop
//! compiles without a solver, a measured table replaces an analytic model without touching
//! the wiring, and evaluation survives garbage.

const std = @import("std");
const base = @import("base");
const spec_mod = @import("spec.zig");
const storage_mod = @import("storage.zig");
const plan_mod = @import("plan.zig");
const compile_mod = @import("compile.zig");
const Diagnostic = @import("diagnostic.zig").Diagnostic;
const components_mod = @import("components");

const Spec = spec_mod.Spec;
const Plan = plan_mod.Plan;
const Workspace = plan_mod.Workspace;

/// Comfortably larger than any scenario plan; the exact size is `bytesNeeded(spec)`.
const test_plan_bytes = 256 * 1024;
const test_ws_bytes = 8 * 1024;

/// Gravity pulls a body down along its axis while air drag (a lift surface at zero angle of
/// attack) resists. Drag depends on velocity, which is a state of the mass: a loop closed by
/// dynamics, so it needs no algebraic solver.
pub const falling_body = struct {
    pub const mass_kg = 2.0;
    pub const area_m2 = 0.5;
    pub const cd = 1.0;

    pub const externals = [_]spec_mod.ExternalSpec{
        .{ .name = "altitude", .dim = base.dim.length },
        .{ .name = "alpha", .dim = base.dim.angle },
        .{ .name = "weight", .dim = base.dim.force },
    };

    pub const connections = [_]spec_mod.ConnectionSpec{
        .{ .from = "altitude", .to = "air.altitude" },
        .{ .from = "air.density", .to = "surface.density" },
        .{ .from = "body.velocity", .to = "surface.airspeed" },
        .{ .from = "alpha", .to = "surface.alpha" },
        .{ .from = "weight", .to = "body.force" },
        .{ .from = "surface.drag", .to = "body.force", .gain = -1.0 },
    };

    pub fn drag_surface() components_mod.lift_surface.Model {
        return .{ .linear = .{
            .area_m2 = area_m2,
            .aspect_ratio = 1.0,
            .cl_alpha_per_rad = 1.0,
            .cd0 = cd,
            .oswald = 1.0,
            .alpha_stall_rad = 0.5,
        } };
    }

    /// Terminal velocity from m g = 1/2 rho v^2 S Cd at sea-level ISA density.
    pub fn terminalVelocity() f64 {
        const rho = base.atmosphere.isa(0.0, 0.0).air.density;
        return @sqrt(2.0 * mass_kg * base.units.g0 / (rho * area_m2 * cd));
    }

    /// The component list is returned by value: a `Spec` only borrows its slices, so the
    /// caller must keep this array alive for as long as the spec is used.
    pub fn components(surface: components_mod.lift_surface.Model) [3]spec_mod.ComponentSpec {
        return .{
            .{ .name = "body", .component = .{ .point_mass = .{
                .rigid = .{ .mass_kg = mass_kg },
            } } },
            .{ .name = "surface", .component = .{ .lift_surface = surface } },
            .{ .name = "air", .component = .{ .air = .{ .isa = .{} } } },
        };
    }

    pub fn spec(comps: *const [3]spec_mod.ComponentSpec) Spec {
        return .{ .components = comps, .externals = &externals, .connections = &connections };
    }
};

fn compileOrFail(spec: *const Spec, buffer: storage_mod.Buffer, diag: *Diagnostic) !*Plan {
    return compile_mod.compileInto(buffer, spec, diag) catch |err| {
        std.debug.print("compile failed: {s}\n", .{diag.message()});
        return err;
    };
}

test "a physical feedback loop through a dynamic component compiles" {
    var plan_storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const comps = falling_body.components(falling_body.drag_surface());
    const spec = falling_body.spec(&comps);
    const plan = try compileOrFail(&spec, &plan_storage, &diag);

    try std.testing.expectEqual(@as(u16, 3), plan.n_nodes);
    try std.testing.expectEqual(@as(u16, 2), plan.n_states);
    try std.testing.expectEqual(@as(u16, 3 + 2 + 2 + 2), plan.n_signals);
    // The air must be computed before the surface that needs its density.
    var air_pos: usize = 99;
    var surface_pos: usize = 99;
    for (plan.order[0..plan.n_nodes], 0..) |ni, pos| {
        if (plan.nodes[ni].name.eql("air")) air_pos = pos;
        if (plan.nodes[ni].name.eql("surface")) surface_pos = pos;
    }
    try std.testing.expect(air_pos < surface_pos);
}

test "the right-hand side of the falling body matches the hand calculation" {
    var plan_storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const comps = falling_body.components(falling_body.drag_surface());
    const spec = falling_body.spec(&comps);
    const plan = try compileOrFail(&spec, &plan_storage, &diag);
    const ws = Workspace.into(plan, &ws_storage);

    const weight = falling_body.mass_kg * base.units.g0;
    const ext = [_]f64{ 0.0, 0.0, weight };
    var dx: [2]f64 = undefined;

    // At rest there is no drag: acceleration is g.
    const v0 = plan.rhs(&.{ 0.0, 0.0 }, &ext, ws, &dx);
    try std.testing.expectEqual(base.Validity.valid, v0);
    try std.testing.expectApproxEqRel(base.units.g0, dx[plan.stateIndex("body.velocity").?], 1e-12);

    // At terminal velocity the forces cancel.
    const vt = falling_body.terminalVelocity();
    const state = [_]f64{ vt, 0.0 };
    _ = plan.rhs(&state, &ext, ws, &dx);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), dx[0], 1e-9);
    try std.testing.expectApproxEqRel(vt, dx[1], 1e-12); // d(position)/dt = velocity
}

test "swapping the model for measured data leaves the wiring untouched" {
    // "Measured" polar: the analytic drag surface sampled at a few angles of attack.
    const analytic = falling_body.drag_surface();
    const wing = analytic.linear;
    const alphas = [_]f64{ -0.5, -0.25, 0.0, 0.25, 0.5 };
    var cls: [alphas.len]f64 = undefined;
    var cds: [alphas.len]f64 = undefined;
    for (alphas, 0..) |a, i| {
        cls[i] = wing.cl_alpha_per_rad * a;
        cds[i] = wing.cd0 + cls[i] * cls[i] / (std.math.pi * wing.oswald * wing.aspect_ratio);
    }
    const Table = components_mod.lift_surface.Table;
    const measured: components_mod.lift_surface.Model = .{ .polar_table = .{
        .area_m2 = wing.area_m2,
        .cl = try Table.init(&alphas, &cls),
        .cd = try Table.init(&alphas, &cds),
    } };

    var storage_a: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var storage_m: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage_a: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage_m: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const comps_a = falling_body.components(analytic);
    const comps_m = falling_body.components(measured);
    const plan_a = try compileOrFail(&falling_body.spec(&comps_a), &storage_a, &diag);
    const plan_m = try compileOrFail(&falling_body.spec(&comps_m), &storage_m, &diag);
    const ws_a = Workspace.into(plan_a, &ws_storage_a);
    const ws_m = Workspace.into(plan_m, &ws_storage_m);

    // Same connections, same layout, same order: only the model inside one node differs.
    try std.testing.expectEqual(plan_a.n_links, plan_m.n_links);
    try std.testing.expectEqual(plan_a.n_signals, plan_m.n_signals);
    try std.testing.expectEqualSlices(u16, plan_a.order[0..3], plan_m.order[0..3]);
    try std.testing.expectEqualStrings("linear", plan_a.nodes[1].component.modelName());
    try std.testing.expectEqualStrings("polar_table", plan_m.nodes[1].component.modelName());

    // At zero angle of attack the table holds exactly cd0, so the two agree everywhere.
    var rng = base.Rng.init(13);
    var dx_a: [2]f64 = undefined;
    var dx_m: [2]f64 = undefined;
    for (0..500) |_| {
        const state = [_]f64{ rng.range(0.0, 40.0), rng.range(-100.0, 100.0) };
        const ext = [_]f64{ rng.range(0.0, 3000.0), 0.0, 19.6 };
        _ = plan_a.rhs(&state, &ext, ws_a, &dx_a);
        const v = plan_m.rhs(&state, &ext, ws_m, &dx_m);
        try std.testing.expectEqual(base.Validity.valid, v);
        try std.testing.expectApproxEqRel(dx_a[0], dx_m[0], 1e-12);
    }
}

test "leaving the measured range is reported through the whole architecture" {
    const alphas = [_]f64{ -0.1, 0.0, 0.1 };
    const coeffs = [_]f64{ 0.5, 0.5, 0.5 };
    const Table = components_mod.lift_surface.Table;
    const narrow: components_mod.lift_surface.Model = .{ .polar_table = .{
        .area_m2 = 0.5,
        .cl = try Table.init(&alphas, &.{ -0.1, 0.0, 0.1 }),
        .cd = try Table.init(&alphas, &coeffs),
    } };
    var plan_storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const comps = falling_body.components(narrow);
    const spec = falling_body.spec(&comps);
    const plan = try compileOrFail(&spec, &plan_storage, &diag);
    const ws = Workspace.into(plan, &ws_storage);

    var dx: [2]f64 = undefined;
    const inside = plan.rhs(&.{ 5.0, 0.0 }, &.{ 0.0, 0.05, 19.6 }, ws, &dx);
    try std.testing.expectEqual(base.Validity.valid, inside);
    const outside = plan.rhs(&.{ 5.0, 0.0 }, &.{ 0.0, 0.4, 19.6 }, ws, &dx);
    try std.testing.expectEqual(base.Validity.beyond_envelope, outside);
    const surface = plan.nodeIndex("surface").?;
    try std.testing.expectEqual(base.Validity.beyond_envelope, ws.validity[surface]);
    try std.testing.expectEqual(base.Validity.valid, ws.validity[plan.nodeIndex("air").?]);
}

test "evaluation is total: garbage in gives finite signals and an invalid flag" {
    var plan_storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const comps = falling_body.components(falling_body.drag_surface());
    const spec = falling_body.spec(&comps);
    const plan = try compileOrFail(&spec, &plan_storage, &diag);
    const ws = Workspace.into(plan, &ws_storage);

    const nan = std.math.nan(f64);
    const inf = std.math.inf(f64);
    const bad_values = [_]f64{ nan, inf, -inf, 1e308 };
    var dx: [2]f64 = undefined;

    for (bad_values) |bad| {
        const states = [_][2]f64{ .{ bad, 0.0 }, .{ 1.0, bad }, .{ 5.0, 5.0 } };
        const exts = [_][3]f64{
            .{ bad, 0.0, 1.0 },
            .{ 0.0, bad, 1.0 },
            .{ 0.0, 0.0, bad },
            .{ 0.0, 0.0, 19.6 },
        };
        for (states) |x| {
            for (exts) |e| {
                const v = plan.rhs(&x, &e, ws, &dx);
                // Component outputs and derivatives are always finite.
                for (ws.signals[plan.n_external_slots..plan.n_signals]) |s| {
                    try std.testing.expect(std.math.isFinite(s));
                }
                for (dx) |d| try std.testing.expect(std.math.isFinite(d));
                // Any non-finite input must be flagged, never silently absorbed.
                var poisoned = false;
                for (x ++ e) |value| poisoned = poisoned or !std.math.isFinite(value);
                if (poisoned) try std.testing.expectEqual(base.Validity.invalid, v);
            }
        }
    }
    // A poisoned state is flagged, not hidden.
    const poisoned = plan.rhs(&.{ nan, 0.0 }, &.{ 0.0, 0.0, 19.6 }, ws, &dx);
    try std.testing.expectEqual(base.Validity.invalid, poisoned);
}

test "random architectures of gains evaluate to the product of their gains" {
    // A property test of the plan machinery: a random chain, declared in random order,
    // must evaluate to x * k1 * k2 * ... regardless of declaration order.
    var plan_storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var rng = base.Rng.init(2468);
    for (0..200) |_| {
        const n = 1 + rng.below(20);
        var order: [20]u32 = undefined;
        for (0..n) |i| order[i] = @intCast(i);
        for (0..n) |i| std.mem.swap(u32, &order[i], &order[rng.below(n)]);

        var names: [20][8]u8 = undefined;
        var name_len: [20]usize = undefined;
        var specs: [20]spec_mod.ComponentSpec = undefined;
        var gains: [20]f64 = undefined;
        var conns: [20]spec_mod.ConnectionSpec = undefined;
        var from_buf: [20][12]u8 = undefined;
        var to_buf: [20][12]u8 = undefined;

        for (0..n) |i| gains[i] = rng.range(0.5, 1.5);
        // Component `order[slot]` holds chain position `slot`'s gain in declaration slot.
        for (0..n) |slot| {
            const label = std.fmt.bufPrint(&names[slot], "g{d}", .{slot}) catch unreachable;
            name_len[slot] = label.len;
            specs[slot] = .{
                .name = names[slot][0..name_len[slot]],
                .component = .{ .gain = .{ .linear = .{ .k = gains[slot] } } },
            };
        }
        // Chain follows the shuffled order: order[0] -> order[1] -> ...
        var expected: f64 = 3.0;
        var n_conn: usize = 0;
        const first = std.fmt.bufPrint(&to_buf[0], "g{d}.x", .{order[0]}) catch unreachable;
        conns[0] = .{ .from = "x", .to = first };
        n_conn = 1;
        for (0..n) |i| expected *= gains[order[i]];
        for (1..n) |i| {
            const f = std.fmt.bufPrint(&from_buf[i], "g{d}.y", .{order[i - 1]}) catch unreachable;
            const t = std.fmt.bufPrint(&to_buf[i], "g{d}.x", .{order[i]}) catch unreachable;
            conns[n_conn] = .{ .from = f, .to = t };
            n_conn += 1;
        }

        const spec: Spec = .{
            .components = specs[0..n],
            .externals = &.{.{ .name = "x", .dim = base.dim.dimensionless }},
            .connections = conns[0..n_conn],
        };
        var diag: Diagnostic = .{};
        const plan = try compileOrFail(&spec, &plan_storage, &diag);
        const ws = Workspace.into(plan, &ws_storage);
        _ = plan.evaluate(&.{}, &.{3.0}, ws);
        var tail_buf: [12]u8 = undefined;
        const tail = std.fmt.bufPrint(&tail_buf, "g{d}.y", .{order[n - 1]}) catch unreachable;
        try std.testing.expectApproxEqRel(expected, ws.signals[plan.signalIndex(tail).?], 1e-12);
    }
}

/// A small 3D aircraft: two wings, one thruster, air, one airflow station per aero surface.
/// Gravity is off so the test isolates the aerodynamic and propulsive forces.
pub const aircraft = struct {
    pub const mass_kg = 2.0;
    pub const area_m2 = 1.2;
    pub const thrust_n = 40.0;
    pub const v_max_ms = 60.0;
    pub const density = 1.225;
    pub const velocity = base.Vec3{ .x = 30.0, .z = 2.0 };

    pub const externals = [_]spec_mod.ExternalSpec{
        .{ .name = "altitude", .dim = base.dim.length },
        .{ .name = "throttle", .dim = base.dim.dimensionless },
    };

    const initial = components_mod.body.initialState(.{ .velocity_body = velocity });

    pub fn wingModel() components_mod.wing.Model {
        return .{ .linear = .{
            .area_m2 = area_m2,
            .aspect_ratio = 8.0,
            .cl_alpha_per_rad = 5.0,
            .cd0 = 0.02,
            .oswald = 0.9,
            .alpha_stall_rad = 0.3,
        } };
    }

    pub fn components() [8]spec_mod.ComponentSpec {
        const left: spec_mod.Placement = .{ .position_m = .{ .y = 1.0 } };
        const right: spec_mod.Placement = .{ .position_m = .{ .y = -1.0 } };
        return .{
            .{
                .name = "body",
                .component = .{ .body = .{ .six_dof = .{
                    .mass_kg = mass_kg,
                    .inertia_kg_m2 = .{ .x = 0.5, .y = 1.0, .z = 1.5 },
                    .gravity_m_s2 = 0.0,
                } } },
                .initial_state = &initial,
            },
            .{
                .name = "air",
                .component = .{ .air = .{ .fixed = .{
                    .density_kg_m3 = density,
                    .speed_of_sound_ms = 340.0,
                } } },
            },
            .{
                .name = "left_station",
                .component = .{ .airflow = .{ .station = .{} } },
                .placement = left,
            },
            .{ .name = "left_wing", .component = .{ .wing = wingModel() }, .placement = left },
            .{
                .name = "right_station",
                .component = .{ .airflow = .{ .station = .{} } },
                .placement = right,
            },
            .{ .name = "right_wing", .component = .{ .wing = wingModel() }, .placement = right },
            .{ .name = "nose_station", .component = .{ .airflow = .{ .station = .{} } } },
            .{
                .name = "motor",
                .component = .{ .thruster = .{ .linear_decay = .{
                    .static_thrust_n = thrust_n,
                    .max_speed_ms = v_max_ms,
                } } },
            },
        };
    }

    pub const connections = [_]spec_mod.ConnectionSpec{
        .{ .from = "altitude", .to = "air.altitude" },
        .{ .from = "body.velocity_body", .to = "left_station.velocity_body" },
        .{ .from = "body.rate_body", .to = "left_station.rate_body" },
        .{ .from = "body.velocity_body", .to = "right_station.velocity_body" },
        .{ .from = "body.rate_body", .to = "right_station.rate_body" },
        .{ .from = "body.velocity_body", .to = "nose_station.velocity_body" },
        .{ .from = "body.rate_body", .to = "nose_station.rate_body" },
        .{ .from = "left_station.alpha", .to = "left_wing.alpha" },
        .{ .from = "left_station.beta", .to = "left_wing.beta" },
        .{ .from = "left_station.airspeed", .to = "left_wing.airspeed" },
        .{ .from = "right_station.alpha", .to = "right_wing.alpha" },
        .{ .from = "right_station.beta", .to = "right_wing.beta" },
        .{ .from = "right_station.airspeed", .to = "right_wing.airspeed" },
        .{ .from = "air.density", .to = "left_wing.density" },
        .{ .from = "air.density", .to = "right_wing.density" },
        .{ .from = "nose_station.airspeed", .to = "motor.airspeed" },
        .{ .from = "throttle", .to = "motor.throttle" },
        .{ .from = "left_wing.wrench", .to = "body.mount", .attachment = .rigid },
        .{ .from = "right_wing.wrench", .to = "body.mount", .attachment = .rigid },
        .{ .from = "motor.wrench", .to = "body.mount", .attachment = .rigid },
    };

    pub fn spec(comps: *const [8]spec_mod.ComponentSpec) Spec {
        return .{ .components = comps, .externals = &externals, .connections = &connections };
    }

    pub const forces = struct { lift: f64, drag: f64, thrust: f64 };

    pub fn forcesAt(throttle: f64) forces {
        const speed = velocity.norm();
        const alpha = std.math.atan2(velocity.z, velocity.x);
        const q_s = 0.5 * density * speed * speed * area_m2;
        const cl = 5.0 * alpha;
        const cd = 0.02 + cl * cl / (std.math.pi * 0.9 * 8.0);
        return .{
            .lift = q_s * cl,
            .drag = q_s * cd,
            .thrust = throttle * thrust_n * @max(0.0, 1.0 - speed / v_max_ms),
        };
    }
};

test "a 3D aircraft: local airflow drives wings whose forces reach the body" {
    var plan_storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const comps = aircraft.components();
    const plan = try compileOrFail(&aircraft.spec(&comps), &plan_storage, &diag);
    const ws = Workspace.into(plan, &ws_storage);

    try std.testing.expectEqual(@as(u16, 11), plan.n_nodes);
    try std.testing.expectEqual(@as(u16, 13), plan.n_states);

    var x: [13]f64 = undefined;
    plan.copyInitialState(&x);
    var dx: [13]f64 = undefined;
    const externals = [_]f64{ 0.0, 1.0 };
    const validity = plan.rhs(&x, &externals, ws, &dx);
    try std.testing.expectEqual(base.Validity.valid, validity);

    // Both stations see the same flow, and the wing at each station consumes it.
    const alpha = std.math.atan2(aircraft.velocity.z, aircraft.velocity.x);
    const speed = aircraft.velocity.norm();
    const alpha_stations = [_][]const u8{
        "left_station.alpha",
        "right_station.alpha",
        "nose_station.alpha",
    };
    for (alpha_stations) |name| {
        try std.testing.expectApproxEqAbs(alpha, ws.signals[plan.signalIndex(name).?], 1e-12);
    }
    const beta_stations = [_][]const u8{
        "left_station.beta",
        "right_station.beta",
        "nose_station.beta",
    };
    for (beta_stations) |name| {
        const beta = ws.signals[plan.signalIndex(name).?];
        try std.testing.expectApproxEqAbs(@as(f64, 0.0), beta, 1e-12);
    }
    for ([_][]const u8{ "left_station.airspeed", "right_station.airspeed" }) |name| {
        try std.testing.expectApproxEqAbs(speed, ws.signals[plan.signalIndex(name).?], 1e-12);
    }

    const f = aircraft.forcesAt(1.0);
    const left = plan.signalSlots("left_wing.wrench").?;
    try std.testing.expectApproxEqRel(-f.drag, ws.signals[left.index], 1e-12);
    try std.testing.expectApproxEqRel(-f.lift, ws.signals[left.index + 2], 1e-12);
    const motor = plan.signalSlots("motor.wrench").?;
    try std.testing.expectApproxEqRel(f.thrust, ws.signals[motor.index], 1e-12);

    // Gravity is off: Newton-Euler reduces to F/m in the body frame.
    const v = plan.stateIndex("body.velocity_body").?;
    try std.testing.expectApproxEqRel(
        (f.thrust - 2.0 * f.drag) / aircraft.mass_kg,
        dx[v],
        1e-12,
    );
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), dx[v + 1], 1e-12);
    try std.testing.expectApproxEqRel(-f.lift, dx[v + 2], 1e-12);

    // Symmetry: the two wings cancel their roll moments and the motor is on the axis.
    const rate = plan.stateIndex("body.rate_body").?;
    for (dx[rate..][0..3]) |d| try std.testing.expectApproxEqAbs(@as(f64, 0.0), d, 1e-12);

    // A level body: position rate is body velocity and the attitude does not move.
    const position = plan.stateIndex("body.position_world").?;
    for (0..3) |i| try std.testing.expectEqual(x[v + i], dx[position + i]);
    const attitude = plan.stateIndex("body.attitude").?;
    for (dx[attitude..][0..4]) |d| try std.testing.expectEqual(@as(f64, 0.0), d);
}

test "the 3D aircraft stays total under non-finite inputs" {
    var plan_storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const comps = aircraft.components();
    const plan = try compileOrFail(&aircraft.spec(&comps), &plan_storage, &diag);
    const ws = Workspace.into(plan, &ws_storage);

    const bad_values = [_]f64{ std.math.nan(f64), std.math.inf(f64), -std.math.inf(f64), 1e308 };
    var dx: [13]f64 = undefined;
    for (bad_values) |bad| {
        var x: [13]f64 = undefined;
        plan.copyInitialState(&x);
        const cases = [_]struct { x: [13]f64, ext: [2]f64 }{
            .{ .x = x, .ext = .{ bad, 1.0 } },
            .{ .x = x, .ext = .{ 0.0, bad } },
            .{ .x = x, .ext = .{ 0.0, 1.0 } },
            .{ .x = blk: {
                var poisoned = x;
                poisoned[0] = bad;
                break :blk poisoned;
            }, .ext = .{ 0.0, 1.0 } },
        };
        for (cases) |c| {
            _ = plan.rhs(&c.x, &c.ext, ws, &dx);
            for (ws.signals[plan.n_external_slots..plan.n_signals]) |s| {
                try std.testing.expect(std.math.isFinite(s));
            }
            for (dx) |d| try std.testing.expect(std.math.isFinite(d));
        }
    }
}
