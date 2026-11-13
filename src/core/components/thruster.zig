//! Thruster: a propeller, a fan or a water jet delivering thrust along the vehicle axis.
//!
//! Inputs   airspeed [m/s] (speed of the vehicle through the fluid), throttle [-] in 0..1
//! Outputs  thrust [N]
//!
//! Models
//!   linear_decay  T = throttle * T0 * (1 - V / Vmax): the usual first estimate of a
//!                 fixed-pitch propeller from its static thrust and its zero-thrust speed
//!   speed_table   full-throttle thrust versus speed measured on a test bench or taken
//!                 from a detailed propeller simulation, scaled by the throttle
//!
//! Validity: throttle outside 0..1 is clamped and flagged; a speed beyond the end of the
//! model (Vmax, last table point) gives the end value flagged `beyond_envelope`; a
//! negative airspeed is `invalid`.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const contract = @import("contract.zig");

pub const Table = base.interp.Table1D(base.limits.max_table_points);

pub const Model = union(enum) {
    linear_decay: LinearDecay,
    speed_table: SpeedTable,

    pub const In = struct { airspeed: f64, throttle: f64 };
    pub const Out = struct { thrust: f64 };
    pub const State = struct {};

    pub const signature = contract.signature(Model, .{
        .in = .{
            .airspeed = contract.scalar(base.dim.speed),
            .throttle = contract.scalar(base.dim.dimensionless),
        },
        .out = .{ .thrust = contract.scalar(base.dim.force) },
        .feedthrough = true,
    });

    pub fn validate(self: *const Model) contract.ParamCheck {
        return switch (self.*) {
            inline else => |*m| m.validate(),
        };
    }

    pub fn outputs(self: *const Model, x: State, u: In) contract.Evaluated(Out) {
        return switch (self.*) {
            inline else => |*m| m.outputs(x, u),
        };
    }
};

const impossible: contract.Evaluated(Model.Out) = .{
    .value = .{ .thrust = 0.0 },
    .validity = .invalid,
};

/// Throttle clamped into 0..1 and the validity that clamping implies.
fn throttleOf(u: Model.In) struct { value: f64, validity: base.Validity } {
    const clamped = std.math.clamp(u.throttle, 0.0, 1.0);
    return .{
        .value = clamped,
        .validity = if (clamped == u.throttle) .valid else .beyond_envelope,
    };
}

pub const LinearDecay = struct {
    static_thrust_n: f64,
    max_speed_ms: f64,

    pub fn validate(self: *const LinearDecay) contract.ParamCheck {
        if (!contract.isPositive(self.static_thrust_n)) return "static_thrust_n must be positive";
        if (!contract.isPositive(self.max_speed_ms)) return "max_speed_ms must be positive";
        return null;
    }

    pub fn outputs(
        self: *const LinearDecay,
        x: Model.State,
        u: Model.In,
    ) contract.Evaluated(Model.Out) {
        _ = x;
        contract.assertFiniteFields(u);
        assert(self.static_thrust_n > 0.0 and self.max_speed_ms > 0.0);
        if (u.airspeed < 0.0) return impossible;

        const throttle = throttleOf(u);
        const speed_ratio = u.airspeed / self.max_speed_ms;
        const full_throttle = self.static_thrust_n * @max(0.0, 1.0 - speed_ratio);
        const speed_validity: base.Validity = if (speed_ratio > 1.0) .beyond_envelope else .valid;
        return .{
            .value = .{ .thrust = throttle.value * full_throttle },
            .validity = throttle.validity.worst(speed_validity),
        };
    }
};

pub const SpeedTable = struct {
    /// Full-throttle thrust [N] versus airspeed [m/s].
    thrust_n: Table,

    pub fn validate(self: *const SpeedTable) contract.ParamCheck {
        for (self.thrust_n.ys[0..self.thrust_n.len]) |t| {
            if (t < 0.0) return "thrust table contains a negative thrust";
        }
        if (self.thrust_n.xMin() < 0.0) return "thrust table starts at a negative airspeed";
        return null;
    }

    pub fn outputs(
        self: *const SpeedTable,
        x: Model.State,
        u: Model.In,
    ) contract.Evaluated(Model.Out) {
        _ = x;
        contract.assertFiniteFields(u);
        if (u.airspeed < 0.0) return impossible;

        const throttle = throttleOf(u);
        const full_throttle = self.thrust_n.lookup(u.airspeed);
        return .{
            .value = .{ .thrust = throttle.value * full_throttle.value },
            .validity = throttle.validity.worst(full_throttle.validity),
        };
    }
};

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

fn sampleThruster() LinearDecay {
    return .{ .static_thrust_n = 40.0, .max_speed_ms = 60.0 };
}

test "linear decay thrust at known points" {
    const m: Model = .{ .linear_decay = sampleThruster() };
    const static = m.outputs(.{}, .{ .airspeed = 0.0, .throttle = 1.0 });
    try std.testing.expectEqual(@as(f64, 40.0), static.value.thrust);
    try std.testing.expectEqual(base.Validity.valid, static.validity);
    const half = m.outputs(.{}, .{ .airspeed = 30.0, .throttle = 1.0 });
    try std.testing.expectApproxEqAbs(@as(f64, 20.0), half.value.thrust, 1e-12);
    const at_vmax = m.outputs(.{}, .{ .airspeed = 60.0, .throttle = 1.0 });
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), at_vmax.value.thrust, 1e-12);
    try std.testing.expectEqual(base.Validity.valid, at_vmax.validity);
}

test "thrust is linear in the throttle and never negative" {
    const m: Model = .{ .linear_decay = sampleThruster() };
    var rng = base.Rng.init(4);
    for (0..2000) |_| {
        const v = rng.range(0.0, 100.0);
        const k = rng.range(0.0, 1.0);
        const full = m.outputs(.{}, .{ .airspeed = v, .throttle = 1.0 }).value.thrust;
        const part = m.outputs(.{}, .{ .airspeed = v, .throttle = k }).value.thrust;
        try std.testing.expectApproxEqAbs(k * full, part, 1e-12);
        try std.testing.expect(part >= 0.0);
    }
}

test "out of range throttle and speed are flagged" {
    const m: Model = .{ .linear_decay = sampleThruster() };
    const over = m.outputs(.{}, .{ .airspeed = 10.0, .throttle = 1.5 });
    try std.testing.expectEqual(base.Validity.beyond_envelope, over.validity);
    const full = m.outputs(.{}, .{ .airspeed = 10.0, .throttle = 1.0 });
    try std.testing.expectEqual(full.value.thrust, over.value.thrust);
    const fast = m.outputs(.{}, .{ .airspeed = 90.0, .throttle = 1.0 });
    try std.testing.expectEqual(base.Validity.beyond_envelope, fast.validity);
    try std.testing.expectEqual(@as(f64, 0.0), fast.value.thrust);
    const backwards = m.outputs(.{}, .{ .airspeed = -1.0, .throttle = 1.0 });
    try std.testing.expectEqual(base.Validity.invalid, backwards.validity);
}

fn sampledTable() !SpeedTable {
    // Samples the linear model at 6 speeds: it is linear, so a table reproduces it exactly.
    const t = sampleThruster();
    var vs: [6]f64 = undefined;
    var ts: [6]f64 = undefined;
    for (0..6) |i| {
        vs[i] = t.max_speed_ms * @as(f64, @floatFromInt(i)) / 5.0;
        ts[i] = t.static_thrust_n * (1.0 - vs[i] / t.max_speed_ms);
    }
    return .{ .thrust_n = try Table.init(&vs, &ts) };
}

test "a measured table reproduces the analytic model it was sampled from" {
    const analytic: Model = .{ .linear_decay = sampleThruster() };
    const measured: Model = .{ .speed_table = try sampledTable() };
    var rng = base.Rng.init(6);
    for (0..2000) |_| {
        const u: Model.In = .{ .airspeed = rng.range(0.0, 60.0), .throttle = rng.range(0.0, 1.0) };
        const a = analytic.outputs(.{}, u);
        const b = measured.outputs(.{}, u);
        try std.testing.expectApproxEqAbs(a.value.thrust, b.value.thrust, 1e-9);
        try std.testing.expectEqual(base.Validity.valid, b.validity);
    }
}

test "beyond the last table point the table flags the result" {
    const measured: Model = .{ .speed_table = try sampledTable() };
    const r = measured.outputs(.{}, .{ .airspeed = 75.0, .throttle = 1.0 });
    try std.testing.expectEqual(base.Validity.beyond_envelope, r.validity);
    try std.testing.expect(std.math.isFinite(r.value.thrust));
}

test "thruster parameters are validated" {
    const bad: Model = .{ .linear_decay = .{ .static_thrust_n = 0.0, .max_speed_ms = 10.0 } };
    try std.testing.expect(bad.validate() != null);
    var table = try sampledTable();
    try std.testing.expectEqual(
        @as(contract.ParamCheck, null),
        (Model{ .speed_table = table }).validate(),
    );
    table.thrust_n.ys[2] = -5.0;
    try std.testing.expect((Model{ .speed_table = table }).validate() != null);
}

test "every thruster model honors the component contract" {
    contract.assertKind(Model);
    var rng = base.Rng.init(41);
    const models = [_]Model{
        .{ .linear_decay = sampleThruster() },
        .{ .speed_table = try sampledTable() },
    };
    for (&models) |*m| try contract.expectKindContract(Model, m, &rng, 2000);
}
