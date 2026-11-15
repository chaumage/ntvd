//! Lift surface: a wing, a foil, a tail or a rudder moving through a fluid.
//!
//! Inputs   airspeed [m/s], alpha [rad] (angle of attack), density [kg/m^3]
//! Outputs  lift [N] (perpendicular to the flow), drag [N] (along the flow, positive)
//! The same kind serves an aircraft wing in air and a hydrofoil in water: only the
//! density supplied by the medium changes.
//!
//! Models, from first estimate to measured data
//!   linear        CL = CL0 + CLa * alpha (limited to the stall angle),
//!                 CD = CD0 + CL^2 / (pi * e * AR)
//!   polar_table   CL(alpha) and CD(alpha) from a wind-tunnel, towing-tank or CFD campaign
//!
//! Validity: |alpha| above the stall angle (linear) or outside the measured alpha range
//! (polar_table) gives a clamped result flagged `beyond_envelope`. Negative airspeed or
//! density gives zero forces flagged `invalid`.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const contract = @import("contract.zig");

pub const Table = base.interp.Table1D(base.limits.max_table_points);

pub const Model = union(enum) {
    linear: Linear,
    polar_table: PolarTable,

    pub const In = struct { airspeed: f64, alpha: f64, density: f64 };
    pub const Out = struct { lift: f64, drag: f64 };
    pub const State = struct {};

    pub const signature = contract.signature(Model, .{
        .in = .{
            .airspeed = contract.scalar(base.dim.speed),
            .alpha = contract.scalar(base.dim.angle),
            .density = contract.scalar(base.dim.density),
        },
        .out = .{
            .lift = contract.scalar(base.dim.force),
            .drag = contract.scalar(base.dim.force),
        },
        .feedthrough = true,
    });

    pub fn validate(self: *const Model) contract.ParamCheck {
        return switch (self.*) {
            inline else => |*m| m.validate(),
        };
    }

    pub fn outputs(
        self: *const Model,
        x: State,
        u: In,
        ctx: contract.Context,
    ) contract.Evaluated(Out) {
        return switch (self.*) {
            inline else => |*m| m.outputs(x, u, ctx),
        };
    }
};

const impossible: contract.Evaluated(Model.Out) = .{
    .value = .{ .lift = 0.0, .drag = 0.0 },
    .validity = .invalid,
};

/// Dynamic pressure times reference area, q * S.
fn dynamicForce(area: f64, u: Model.In) f64 {
    return 0.5 * u.density * u.airspeed * u.airspeed * area;
}

pub const Linear = struct {
    area_m2: f64,
    aspect_ratio: f64,
    cl_alpha_per_rad: f64,
    cd0: f64,
    /// Span efficiency factor e of the induced drag, 1.0 for an elliptic lift distribution.
    oswald: f64,
    alpha_stall_rad: f64,
    cl0: f64 = 0.0,

    pub fn validate(self: *const Linear) contract.ParamCheck {
        if (!contract.isPositive(self.area_m2)) return "area_m2 must be positive";
        if (!contract.isPositive(self.aspect_ratio)) return "aspect_ratio must be positive";
        if (!contract.isPositive(self.cl_alpha_per_rad)) return "cl_alpha_per_rad must be positive";
        if (!contract.isNonNegative(self.cd0)) return "cd0 must be non-negative";
        if (!contract.isPositive(self.oswald)) return "oswald must be positive";
        if (self.oswald > 1.2) return "oswald above 1.2 is not physical for a single surface";
        if (!contract.isPositive(self.alpha_stall_rad)) return "alpha_stall_rad must be positive";
        if (!contract.isFinite(self.cl0)) return "cl0 must be finite";
        return null;
    }

    pub fn outputs(
        self: *const Linear,
        x: Model.State,
        u: Model.In,
        ctx: contract.Context,
    ) contract.Evaluated(Model.Out) {
        _ = x;
        _ = ctx;
        contract.assertFiniteFields(u);
        assert(self.area_m2 > 0.0 and self.aspect_ratio > 0.0 and self.oswald > 0.0);
        if (u.airspeed < 0.0 or u.density < 0.0) return impossible;

        const stall = self.alpha_stall_rad;
        const alpha = std.math.clamp(u.alpha, -stall, stall);
        const q_s = dynamicForce(self.area_m2, u);
        const cl = self.cl0 + self.cl_alpha_per_rad * alpha;
        const cd = self.cd0 + cl * cl / (std.math.pi * self.oswald * self.aspect_ratio);
        return .{
            .value = .{ .lift = q_s * cl, .drag = q_s * cd },
            .validity = if (@abs(u.alpha) > stall) .beyond_envelope else .valid,
        };
    }
};

pub const PolarTable = struct {
    area_m2: f64,
    /// Lift coefficient versus angle of attack [rad].
    cl: Table,
    /// Drag coefficient versus angle of attack [rad].
    cd: Table,

    pub fn validate(self: *const PolarTable) contract.ParamCheck {
        if (!contract.isPositive(self.area_m2)) return "area_m2 must be positive";
        for (self.cd.ys[0..self.cd.len]) |c| {
            if (c < 0.0) return "cd table contains a negative drag coefficient";
        }
        return null;
    }

    pub fn outputs(
        self: *const PolarTable,
        x: Model.State,
        u: Model.In,
        ctx: contract.Context,
    ) contract.Evaluated(Model.Out) {
        _ = x;
        _ = ctx;
        contract.assertFiniteFields(u);
        assert(self.area_m2 > 0.0);
        if (u.airspeed < 0.0 or u.density < 0.0) return impossible;

        const cl = self.cl.lookup(u.alpha);
        const cd = self.cd.lookup(u.alpha);
        const q_s = dynamicForce(self.area_m2, u);
        return .{
            .value = .{ .lift = q_s * cl.value, .drag = q_s * cd.value },
            .validity = cl.validity.worst(cd.validity),
        };
    }
};

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

fn sampleWing() Linear {
    return .{
        .area_m2 = 1.2,
        .aspect_ratio = 8.0,
        .cl_alpha_per_rad = 5.0,
        .cd0 = 0.02,
        .oswald = 0.9,
        .alpha_stall_rad = 0.3,
    };
}

const sample_flow: Model.In = .{ .airspeed = 30.0, .alpha = 0.05, .density = 1.225 };

test "linear wing against a hand calculation" {
    const m: Model = .{ .linear = sampleWing() };
    const r = m.outputs(.{}, sample_flow, .{});
    // q = 0.5 * 1.225 * 30^2 = 551.25 Pa;  CL = 5 * 0.05 = 0.25
    // lift = 551.25 * 1.2 * 0.25 = 165.375 N
    // CD = 0.02 + 0.25^2 / (pi * 0.9 * 8) = 0.0227631...;  drag = 551.25 * 1.2 * CD
    try std.testing.expectApproxEqAbs(@as(f64, 165.375), r.value.lift, 1e-9);
    const cd = 0.02 + 0.0625 / (std.math.pi * 0.9 * 8.0);
    try std.testing.expectApproxEqRel(551.25 * 1.2 * cd, r.value.drag, 1e-12);
    try std.testing.expectEqual(base.Validity.valid, r.validity);
}

test "an elliptic wing has induced drag CL^2 / (pi AR)" {
    var wing = sampleWing();
    wing.cd0 = 0.0;
    wing.oswald = 1.0;
    const m: Model = .{ .linear = wing };
    const u: Model.In = .{ .airspeed = 25.0, .alpha = 0.1, .density = 1.1 };
    const r = m.outputs(.{}, u, .{});
    const cl = r.value.lift / (0.5 * u.density * u.airspeed * u.airspeed * wing.area_m2);
    const cdi = r.value.drag / (0.5 * u.density * u.airspeed * u.airspeed * wing.area_m2);
    try std.testing.expectApproxEqRel(cl * cl / (std.math.pi * wing.aspect_ratio), cdi, 1e-12);
}

test "forces scale with area, density and speed squared" {
    var rng = base.Rng.init(3);
    const base_wing = sampleWing();
    for (0..1000) |_| {
        const u: Model.In = .{
            .airspeed = rng.range(1.0, 80.0),
            .alpha = rng.range(-0.25, 0.25),
            .density = rng.range(0.5, 1100.0),
        };
        const k = rng.range(0.5, 4.0);
        const one: Model = .{ .linear = base_wing };
        var bigger = base_wing;
        bigger.area_m2 *= k;
        const scaled_area: Model = .{ .linear = bigger };

        const a = one.outputs(.{}, u, .{}).value;
        const b = scaled_area.outputs(.{}, u, .{}).value;
        try std.testing.expectApproxEqRel(k * a.lift, b.lift, 1e-12);
        try std.testing.expectApproxEqRel(k * a.drag, b.drag, 1e-12);

        var faster = u;
        faster.airspeed *= k;
        const c = one.outputs(.{}, faster, .{}).value;
        try std.testing.expectApproxEqRel(k * k * a.lift, c.lift, 1e-12);
    }
}

test "beyond stall the result is clamped and flagged" {
    const m: Model = .{ .linear = sampleWing() };
    var u = sample_flow;
    u.alpha = 0.3;
    const at_stall = m.outputs(.{}, u, .{});
    try std.testing.expectEqual(base.Validity.valid, at_stall.validity);
    u.alpha = 0.6;
    const past = m.outputs(.{}, u, .{});
    try std.testing.expectEqual(base.Validity.beyond_envelope, past.validity);
    try std.testing.expectEqual(at_stall.value.lift, past.value.lift);
    u.alpha = -0.6;
    try std.testing.expectEqual(base.Validity.beyond_envelope, m.outputs(.{}, u, .{}).validity);
}

test "impossible conditions are flagged invalid with zero forces" {
    const m: Model = .{ .linear = sampleWing() };
    var u = sample_flow;
    u.density = -1.0;
    const r = m.outputs(.{}, u, .{});
    try std.testing.expectEqual(base.Validity.invalid, r.validity);
    try std.testing.expectEqual(@as(f64, 0.0), r.value.lift);
    u = sample_flow;
    u.airspeed = -3.0;
    try std.testing.expectEqual(base.Validity.invalid, m.outputs(.{}, u, .{}).validity);
}

/// A polar table sampled from the analytic wing: the "measured" data of these tests.
fn sampledPolar(wing: Linear) !PolarTable {
    const n = 25;
    var alphas: [n]f64 = undefined;
    var cls: [n]f64 = undefined;
    var cds: [n]f64 = undefined;
    for (0..n) |i| {
        const a = -wing.alpha_stall_rad + 2.0 * wing.alpha_stall_rad *
            @as(f64, @floatFromInt(i)) / (n - 1);
        alphas[i] = a;
        cls[i] = wing.cl0 + wing.cl_alpha_per_rad * a;
        cds[i] = wing.cd0 + cls[i] * cls[i] / (std.math.pi * wing.oswald * wing.aspect_ratio);
    }
    return .{
        .area_m2 = wing.area_m2,
        .cl = try Table.init(&alphas, &cls),
        .cd = try Table.init(&alphas, &cds),
    };
}

test "a table sampled from the analytic model gives nearly the same answer" {
    const wing = sampleWing();
    const analytic: Model = .{ .linear = wing };
    const measured: Model = .{ .polar_table = try sampledPolar(wing) };
    var rng = base.Rng.init(8);
    for (0..2000) |_| {
        const u: Model.In = .{
            .airspeed = rng.range(5.0, 60.0),
            .alpha = rng.range(-0.3, 0.3),
            .density = rng.range(0.8, 1.3),
        };
        const a = analytic.outputs(.{}, u, .{});
        const b = measured.outputs(.{}, u, .{});
        try std.testing.expectEqual(base.Validity.valid, b.validity);
        // CL is linear in alpha, so interpolation reproduces it exactly.
        try std.testing.expectApproxEqAbs(a.value.lift, b.value.lift, 1e-9);
        // CD is quadratic: the chord error of a 25-point table is below 1 %.
        try std.testing.expectApproxEqRel(a.value.drag, b.value.drag, 0.01);
    }
}

test "outside the measured alpha range the table flags the result" {
    const measured: Model = .{ .polar_table = try sampledPolar(sampleWing()) };
    var u = sample_flow;
    u.alpha = 0.9;
    try std.testing.expectEqual(
        base.Validity.beyond_envelope,
        measured.outputs(.{}, u, .{}).validity,
    );
}

test "lift surface parameters are validated" {
    const ok: Model = .{ .linear = sampleWing() };
    try std.testing.expectEqual(@as(contract.ParamCheck, null), ok.validate());

    var wing = sampleWing();
    wing.area_m2 = -1.0;
    const negative_area: Model = .{ .linear = wing };
    try std.testing.expectEqualStrings("area_m2 must be positive", negative_area.validate().?);

    wing = sampleWing();
    wing.oswald = 2.0;
    const too_efficient: Model = .{ .linear = wing };
    try std.testing.expect(too_efficient.validate() != null);

    var polar = try sampledPolar(sampleWing());
    polar.cd.ys[3] = -0.01;
    const negative_drag: Model = .{ .polar_table = polar };
    try std.testing.expect(negative_drag.validate() != null);
}

test "every lift surface model honors the component contract" {
    contract.assertKind(Model);
    var rng = base.Rng.init(31);
    const models = [_]Model{
        .{ .linear = sampleWing() },
        .{ .polar_table = try sampledPolar(sampleWing()) },
    };
    for (&models) |*m| try contract.expectKindContract(Model, m, &rng, 2000);
}
