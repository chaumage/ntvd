//! End-to-end architectures that exercise the model layer the way an engineer will.
//!
//! These are the tests that prove the abstractions hold together: a physical feedback loop
//! compiles without a solver, a measured table replaces an analytic model without touching
//! the wiring, and evaluation survives garbage.

const std = @import("std");
const base = @import("base");
const spec_mod = @import("spec.zig");
const plan_mod = @import("plan.zig");
const compile_mod = @import("compile.zig");
const Diagnostic = @import("diagnostic.zig").Diagnostic;
const components_mod = @import("components");

const Spec = spec_mod.Spec;
const Plan = plan_mod.Plan;
const Workspace = plan_mod.Workspace;

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

fn compileOrFail(spec: *const Spec, plan: *Plan) !void {
    var diag: Diagnostic = .{};
    compile_mod.compile(spec, plan, &diag) catch |err| {
        std.debug.print("compile failed: {s}\n", .{diag.message()});
        return err;
    };
}

test "a physical feedback loop through a dynamic component compiles" {
    var plan: Plan = undefined;
    const comps = falling_body.components(falling_body.drag_surface());
    const spec = falling_body.spec(&comps);
    try compileOrFail(&spec, &plan);

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
    var plan: Plan = undefined;
    const comps = falling_body.components(falling_body.drag_surface());
    const spec = falling_body.spec(&comps);
    try compileOrFail(&spec, &plan);

    const weight = falling_body.mass_kg * base.units.g0;
    const ext = [_]f64{ 0.0, 0.0, weight };
    var ws: Workspace = .{};
    var dx: [2]f64 = undefined;

    // At rest there is no drag: acceleration is g.
    const v0 = plan.rhs(&.{ 0.0, 0.0 }, &ext, &ws, &dx);
    try std.testing.expectEqual(base.Validity.valid, v0);
    try std.testing.expectApproxEqRel(base.units.g0, dx[plan.stateIndex("body.velocity").?], 1e-12);

    // At terminal velocity the forces cancel.
    const vt = falling_body.terminalVelocity();
    const state = [_]f64{ vt, 0.0 };
    _ = plan.rhs(&state, &ext, &ws, &dx);
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

    var plan_a: Plan = undefined;
    var plan_m: Plan = undefined;
    const comps_a = falling_body.components(analytic);
    const comps_m = falling_body.components(measured);
    const spec_a = falling_body.spec(&comps_a);
    const spec_m = falling_body.spec(&comps_m);
    try compileOrFail(&spec_a, &plan_a);
    try compileOrFail(&spec_m, &plan_m);

    // Same connections, same layout, same order: only the model inside one node differs.
    try std.testing.expectEqual(plan_a.n_links, plan_m.n_links);
    try std.testing.expectEqual(plan_a.n_signals, plan_m.n_signals);
    try std.testing.expectEqualSlices(u16, plan_a.order[0..3], plan_m.order[0..3]);
    try std.testing.expectEqualStrings("linear", plan_a.nodes[1].component.modelName());
    try std.testing.expectEqualStrings("polar_table", plan_m.nodes[1].component.modelName());

    // At zero angle of attack the table holds exactly cd0, so the two agree everywhere.
    var rng = base.Rng.init(13);
    var ws_a: Workspace = .{};
    var ws_m: Workspace = .{};
    var dx_a: [2]f64 = undefined;
    var dx_m: [2]f64 = undefined;
    for (0..500) |_| {
        const state = [_]f64{ rng.range(0.0, 40.0), rng.range(-100.0, 100.0) };
        const ext = [_]f64{ rng.range(0.0, 3000.0), 0.0, 19.6 };
        _ = plan_a.rhs(&state, &ext, &ws_a, &dx_a);
        const v = plan_m.rhs(&state, &ext, &ws_m, &dx_m);
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
    var plan: Plan = undefined;
    const comps = falling_body.components(narrow);
    const spec = falling_body.spec(&comps);
    try compileOrFail(&spec, &plan);

    var ws: Workspace = .{};
    var dx: [2]f64 = undefined;
    const inside = plan.rhs(&.{ 5.0, 0.0 }, &.{ 0.0, 0.05, 19.6 }, &ws, &dx);
    try std.testing.expectEqual(base.Validity.valid, inside);
    const outside = plan.rhs(&.{ 5.0, 0.0 }, &.{ 0.0, 0.4, 19.6 }, &ws, &dx);
    try std.testing.expectEqual(base.Validity.beyond_envelope, outside);
    const surface = plan.nodeIndex("surface").?;
    try std.testing.expectEqual(base.Validity.beyond_envelope, ws.validity[surface]);
    try std.testing.expectEqual(base.Validity.valid, ws.validity[plan.nodeIndex("air").?]);
}

test "evaluation is total: garbage in gives finite signals and an invalid flag" {
    var plan: Plan = undefined;
    const comps = falling_body.components(falling_body.drag_surface());
    const spec = falling_body.spec(&comps);
    try compileOrFail(&spec, &plan);

    const nan = std.math.nan(f64);
    const inf = std.math.inf(f64);
    const bad_values = [_]f64{ nan, inf, -inf, 1e308 };
    var ws: Workspace = .{};
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
                const v = plan.rhs(&x, &e, &ws, &dx);
                // Component outputs and derivatives are always finite.
                for (ws.signals[plan.n_externals..plan.n_signals]) |s| {
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
    const poisoned = plan.rhs(&.{ nan, 0.0 }, &.{ 0.0, 0.0, 19.6 }, &ws, &dx);
    try std.testing.expectEqual(base.Validity.invalid, poisoned);
}

test "random architectures of gains evaluate to the product of their gains" {
    // A property test of the plan machinery: a random chain, declared in random order,
    // must evaluate to x * k1 * k2 * ... regardless of declaration order.
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
        var plan: Plan = undefined;
        try compileOrFail(&spec, &plan);
        var ws: Workspace = .{};
        _ = plan.evaluate(&.{}, &.{3.0}, &ws);
        var tail_buf: [12]u8 = undefined;
        const tail = std.fmt.bufPrint(&tail_buf, "g{d}.y", .{order[n - 1]}) catch unreachable;
        try std.testing.expectApproxEqRel(expected, ws.signals[plan.signalIndex(tail).?], 1e-12);
    }
}
