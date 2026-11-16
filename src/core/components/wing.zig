//! Wing: a lifting surface in its own natural frame, output as a local wrench.
//!
//! Inputs   alpha [rad], beta [rad], airspeed [m/s], density [kg/m^3]
//! Outputs  wrench [N, N*m] in the local frame, plus lift [N], drag [N], side_force [N]
//!
//! Local axes: x chord-forward, y span-right, z normal-down. A positive angle of attack gives
//! positive lift along local `-z`; drag is positive along local `-x`; a positive sideslip
//! angle gives a side force along local `+y` when `cy_beta_per_rad` is positive. The force
//! part of the wrench is therefore `(-drag, side_force, -lift)` and its moment is zero: a
//! wing has no intrinsic pitching moment in this model. The scalars mirror the existing
//! `lift_surface` kind so both can be compared and either can drive a scalar force path.
//!
//! Models, from first estimate to measured data
//!   linear        CL = CL0 + CLa * alpha (limited to the stall angle),
//!                 CD = CD0 + CL^2 / (pi * e * AR), CY = CY_beta * beta
//!   polar_table   CL(alpha) and CD(alpha) from a wind-tunnel, towing-tank or CFD campaign;
//!                 the side force stays linear in beta
//!
//! Validity: |alpha| above the stall angle (linear) or outside the measured alpha range
//! (polar_table) gives a clamped result flagged `beyond_envelope`. Negative airspeed or
//! density gives zero forces flagged `invalid`.
//!
//! An `airflow` station supplies alpha, beta and airspeed. Give the airflow and the wing the
//! same placement, or they will describe different stations of the same vehicle.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const contract = @import("contract.zig");
const surface = @import("lift_surface.zig");

pub const Table = surface.Table;

pub const Model = union(enum) {
    linear: Linear,
    polar_table: PolarTable,

    pub const In = struct {
        alpha: f64 = 0.0,
        beta: f64 = 0.0,
        airspeed: f64 = 0.0,
        density: f64 = 0.0,
    };
    pub const Out = struct {
        wrench: base.Wrench,
        lift: f64,
        drag: f64,
        side_force: f64,
    };
    pub const State = struct {};

    pub const signature = contract.signature(Model, .{
        .in = .{
            .alpha = contract.scalar(base.dim.angle),
            .beta = contract.scalar(base.dim.angle),
            .airspeed = contract.scalar(base.dim.speed),
            .density = contract.scalar(base.dim.density),
        },
        .out = .{
            .wrench = contract.wrench(.local),
            .lift = contract.scalar(base.dim.force),
            .drag = contract.scalar(base.dim.force),
            .side_force = contract.scalar(base.dim.force),
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
    .value = .{ .wrench = .{}, .lift = 0.0, .drag = 0.0, .side_force = 0.0 },
    .validity = .invalid,
};

/// Dynamic pressure times reference area, q * S.
fn dynamicForce(area: f64, u: Model.In) f64 {
    return 0.5 * u.density * u.airspeed * u.airspeed * area;
}

/// Groups the coefficients of the lifting part into the local wrench and scalars.
fn evaluated(
    force: surface.Model.Out,
    side_force: f64,
    validity: base.Validity,
) contract.Evaluated(Model.Out) {
    if (validity == .invalid) return impossible;
    return .{ .value = .{
        .wrench = .{ .force = .{
            .x = -force.drag,
            .y = side_force,
            .z = -force.lift,
        } },
        .lift = force.lift,
        .drag = force.drag,
        .side_force = side_force,
    }, .validity = validity };
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
    /// Side-force coefficient slope, positive for a force along local `+y` with positive beta.
    cy_beta_per_rad: f64 = 0.0,

    fn lifting(self: *const Linear) surface.Linear {
        return .{
            .area_m2 = self.area_m2,
            .aspect_ratio = self.aspect_ratio,
            .cl_alpha_per_rad = self.cl_alpha_per_rad,
            .cd0 = self.cd0,
            .oswald = self.oswald,
            .alpha_stall_rad = self.alpha_stall_rad,
            .cl0 = self.cl0,
        };
    }

    pub fn validate(self: *const Linear) contract.ParamCheck {
        if (self.lifting().validate()) |message| return message;
        if (!contract.isFinite(self.cy_beta_per_rad)) {
            return "cy_beta_per_rad must be finite";
        }
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
        assert(contract.isFinite(self.cy_beta_per_rad));
        const r = self.lifting().outputs(
            .{},
            .{ .airspeed = u.airspeed, .alpha = u.alpha, .density = u.density },
            .{},
        );
        const side_force = dynamicForce(self.area_m2, u) * self.cy_beta_per_rad * u.beta;
        return evaluated(r.value, side_force, r.validity);
    }
};

pub const PolarTable = struct {
    area_m2: f64,
    /// Lift coefficient versus angle of attack [rad].
    cl: Table,
    /// Drag coefficient versus angle of attack [rad].
    cd: Table,
    /// Side-force coefficient slope, positive for a force along local `+y` with positive beta.
    cy_beta_per_rad: f64 = 0.0,

    fn lifting(self: *const PolarTable) surface.PolarTable {
        return .{ .area_m2 = self.area_m2, .cl = self.cl, .cd = self.cd };
    }

    pub fn validate(self: *const PolarTable) contract.ParamCheck {
        if (self.lifting().validate()) |message| return message;
        if (!contract.isFinite(self.cy_beta_per_rad)) {
            return "cy_beta_per_rad must be finite";
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
        assert(contract.isFinite(self.cy_beta_per_rad));
        const r = self.lifting().outputs(
            .{},
            .{ .airspeed = u.airspeed, .alpha = u.alpha, .density = u.density },
            .{},
        );
        const side_force = dynamicForce(self.area_m2, u) * self.cy_beta_per_rad * u.beta;
        return evaluated(r.value, side_force, r.validity);
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

const sample_flow: Model.In = .{
    .alpha = 0.05,
    .beta = 0.0,
    .airspeed = 30.0,
    .density = 1.225,
};

test "a linear wing matches the hand calculation and builds a local wrench" {
    const m: Model = .{ .linear = sampleWing() };
    const r = m.outputs(.{}, sample_flow, .{});
    const q_s = 0.5 * 1.225 * 30.0 * 30.0 * 1.2;
    const cl = 5.0 * 0.05;
    const cd = 0.02 + cl * cl / (std.math.pi * 0.9 * 8.0);
    const lift = q_s * cl;
    const drag = q_s * cd;

    try std.testing.expectApproxEqAbs(lift, r.value.lift, 1e-9);
    try std.testing.expectApproxEqRel(drag, r.value.drag, 1e-12);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), r.value.side_force, 1e-12);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), r.value.wrench.force.x + drag, 1e-12);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), r.value.wrench.force.y, 1e-12);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), r.value.wrench.force.z + lift, 1e-12);
    try std.testing.expectEqualDeep(base.Wrench.zero.moment, r.value.wrench.moment);
    try std.testing.expectEqual(base.Validity.valid, r.validity);
}

test "positive alpha lifts along local -z and drags along local -x" {
    const m: Model = .{ .linear = sampleWing() };
    const r = m.outputs(.{}, sample_flow, .{});
    try std.testing.expect(r.value.wrench.force.z < 0.0);
    try std.testing.expect(r.value.wrench.force.x < 0.0);
    try std.testing.expect(r.value.lift > 0.0);
    try std.testing.expect(r.value.drag > 0.0);
}

test "positive beta gives a side force along local +y" {
    var wing = sampleWing();
    wing.cy_beta_per_rad = 0.5;
    const m: Model = .{ .linear = wing };
    var u = sample_flow;
    u.beta = 0.1;
    const r = m.outputs(.{}, u, .{});
    const expected = 0.5 * u.density * u.airspeed * u.airspeed * wing.area_m2 *
        wing.cy_beta_per_rad * u.beta;
    try std.testing.expectApproxEqRel(expected, r.value.side_force, 1e-12);
    try std.testing.expectApproxEqRel(expected, r.value.wrench.force.y, 1e-12);
    u.beta = -0.1;
    try std.testing.expect(m.outputs(.{}, u, .{}).value.wrench.force.y < 0.0);
}

test "a wing agrees with lift_surface on lift, drag and validity" {
    const wing = sampleWing();
    const wing_model: Model = .{ .linear = wing };
    const surface_model: surface.Model = .{ .linear = wing.lifting() };
    var rng = base.Rng.init(71);
    for (0..2000) |_| {
        const u: Model.In = .{
            .alpha = rng.range(-0.6, 0.6),
            .beta = rng.range(-0.5, 0.5),
            .airspeed = rng.range(-5.0, 80.0),
            .density = rng.range(-0.5, 1.5),
        };
        const a = wing_model.outputs(.{}, u, .{});
        const b = surface_model.outputs(
            .{},
            .{ .alpha = u.alpha, .airspeed = u.airspeed, .density = u.density },
            .{},
        );
        try std.testing.expectEqual(b.validity, a.validity);
        try std.testing.expectEqual(b.value.lift, a.value.lift);
        try std.testing.expectEqual(b.value.drag, a.value.drag);
        if (a.validity != .invalid) {
            try std.testing.expectEqual(-b.value.drag, a.value.wrench.force.x);
            try std.testing.expectEqual(-b.value.lift, a.value.wrench.force.z);
        }
    }
}

test "impossible conditions clear the whole wrench" {
    var wing = sampleWing();
    wing.cy_beta_per_rad = 1.0;
    const m: Model = .{ .linear = wing };
    var u = sample_flow;
    u.density = -1.0;
    const r = m.outputs(.{}, u, .{});
    try std.testing.expectEqual(base.Validity.invalid, r.validity);
    try std.testing.expectEqualDeep(base.Wrench.zero, r.value.wrench);
    try std.testing.expectEqual(@as(f64, 0.0), r.value.side_force);
    u = sample_flow;
    u.airspeed = -3.0;
    try std.testing.expectEqual(base.Validity.invalid, m.outputs(.{}, u, .{}).validity);
}

test "beyond stall the result is clamped and flagged" {
    const m: Model = .{ .linear = sampleWing() };
    var u = sample_flow;
    u.alpha = 0.6;
    const past = m.outputs(.{}, u, .{});
    try std.testing.expectEqual(base.Validity.beyond_envelope, past.validity);
    u.alpha = 0.3;
    const at_stall = m.outputs(.{}, u, .{});
    try std.testing.expectEqualDeep(at_stall.value.wrench, past.value.wrench);
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
        .cy_beta_per_rad = wing.cy_beta_per_rad,
    };
}

test "a table sampled from the analytic model gives nearly the same answer" {
    const wing = sampleWing();
    const analytic: Model = .{ .linear = wing };
    const measured: Model = .{ .polar_table = try sampledPolar(wing) };
    var rng = base.Rng.init(81);
    for (0..2000) |_| {
        const u: Model.In = .{
            .alpha = rng.range(-0.3, 0.3),
            .beta = rng.range(-0.2, 0.2),
            .airspeed = rng.range(5.0, 60.0),
            .density = rng.range(0.8, 1.3),
        };
        const a = analytic.outputs(.{}, u, .{});
        const b = measured.outputs(.{}, u, .{});
        try std.testing.expectEqual(base.Validity.valid, b.validity);
        try std.testing.expectApproxEqAbs(a.value.lift, b.value.lift, 1e-9);
        try std.testing.expectApproxEqRel(a.value.drag, b.value.drag, 0.01);
        try std.testing.expectEqual(a.value.side_force, b.value.side_force);
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

test "wing parameters are validated" {
    const ok: Model = .{ .linear = sampleWing() };
    try std.testing.expectEqual(@as(contract.ParamCheck, null), ok.validate());

    var wing = sampleWing();
    wing.area_m2 = -1.0;
    const negative_area: Model = .{ .linear = wing };
    try std.testing.expectEqualStrings("area_m2 must be positive", negative_area.validate().?);

    wing = sampleWing();
    wing.cy_beta_per_rad = std.math.nan(f64);
    const bad_slope: Model = .{ .linear = wing };
    try std.testing.expectEqualStrings("cy_beta_per_rad must be finite", bad_slope.validate().?);

    var polar = try sampledPolar(sampleWing());
    polar.cd.ys[3] = -0.01;
    const negative_drag: Model = .{ .polar_table = polar };
    try std.testing.expect(negative_drag.validate() != null);
}

test "every wing model honors the component contract" {
    contract.assertKind(Model);
    var rng = base.Rng.init(91);
    const models = [_]Model{
        .{ .linear = sampleWing() },
        .{ .polar_table = try sampledPolar(sampleWing()) },
    };
    for (&models) |*m| try contract.expectKindContract(Model, m, &rng, 2000);
}
