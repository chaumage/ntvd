//! Airflow at a station: the local flow seen by a component attached to a moving body.
//!
//! Inputs   velocity_body [m/s], rate_body [rad/s], both in the body frame
//! Outputs  alpha [rad], beta [rad], airspeed [m/s]
//!
//! The kind reads its own placement from the compile-time context. With `(r, R)` the
//! placement (origin in the body frame and body<-local rotation) and `omega` the body rate:
//!
//!   v_local  = R^T (v_body + omega x r)
//!   alpha    = atan2(v_local.z, v_local.x)
//!   beta     = atan2(v_local.y, sqrt(v_local.x^2 + v_local.z^2))
//!   airspeed = |v_local|
//!
//! Positive alpha means the local velocity points along `+z` (the nose is above the flight
//! path); positive beta means it points along `+y` (slipping right). The local axes are the
//! component's own: for an aero surface, x chord-forward, y span-right, z normal-down.
//!
//! An aero surface that consumes these outputs must carry the same placement, or the two
//! disagree about which station they describe. The compiler cannot check that rule: verify it
//! when authoring, and see the aircraft scenario for a pinned example.
//!
//! Validity: always valid for a finite state. A zero local velocity gives zero angles and
//! zero airspeed; the model has no parameters.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const contract = @import("contract.zig");

pub const Model = union(enum) {
    station: Station,

    pub const In = struct {
        velocity_body: base.Vec3 = .{},
        rate_body: base.Vec3 = .{},
    };
    pub const Out = struct { alpha: f64, beta: f64, airspeed: f64 };
    pub const State = struct {};

    pub const signature = contract.signature(Model, .{
        .in = .{
            .velocity_body = contract.vec3(base.dim.speed, .velocity, .body),
            .rate_body = contract.vec3(base.dim.angular_rate, .angular_rate, .body),
        },
        .out = .{
            .alpha = contract.scalar(base.dim.angle),
            .beta = contract.scalar(base.dim.angle),
            .airspeed = contract.scalar(base.dim.speed),
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

pub const Station = struct {
    pub fn validate(self: *const Station) contract.ParamCheck {
        _ = self;
        return null;
    }

    pub fn outputs(
        self: *const Station,
        x: Model.State,
        u: Model.In,
        ctx: contract.Context,
    ) contract.Evaluated(Model.Out) {
        _ = self;
        _ = x;
        contract.assertFiniteFields(u);
        assert(ctx.placement.isFinite());

        const position = ctx.placement.position;
        const velocity = u.velocity_body.add(u.rate_body.cross(position));
        const local = ctx.placement.orientation.conjugate().rotate(velocity);
        const horizontal = @sqrt(local.x * local.x + local.z * local.z);
        const airspeed = @sqrt(local.x * local.x + local.y * local.y + local.z * local.z);
        return .{ .value = .{
            .alpha = std.math.atan2(local.z, local.x),
            .beta = std.math.atan2(local.y, horizontal),
            .airspeed = airspeed,
        } };
    }
};

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

const tolerance = 1.0e-12;

fn station() Model {
    return .{ .station = .{} };
}

fn placement(position: base.Vec3, yaw: f64) contract.Context {
    return .{ .placement = .{
        .position = position,
        .orientation = base.Quat.fromEuler(0.0, 0.0, yaw),
    } };
}

test "the local flow gives alpha, beta and airspeed" {
    const m = station();
    const u: Model.In = .{ .velocity_body = .{ .x = 10.0, .y = 2.0, .z = 1.0 } };
    const r = m.outputs(.{}, u, .{});

    const horizontal = @sqrt(10.0 * 10.0 + 1.0 * 1.0);
    const expected_alpha = std.math.atan2(@as(f64, 1.0), @as(f64, 10.0));
    const expected_beta = std.math.atan2(@as(f64, 2.0), horizontal);
    try std.testing.expectApproxEqAbs(expected_alpha, r.value.alpha, tolerance);
    try std.testing.expectApproxEqAbs(expected_beta, r.value.beta, tolerance);
    try std.testing.expectApproxEqAbs(@sqrt(105.0), r.value.airspeed, tolerance);
    try std.testing.expectEqual(base.Validity.valid, r.validity);
}

test "positive signs follow the local axes" {
    const m = station();
    // +z in the local velocity is positive alpha; +y is positive beta.
    const up = m.outputs(.{}, .{ .velocity_body = .{ .x = 1.0, .z = 1.0 } }, .{});
    try std.testing.expect(up.value.alpha > 0.0);
    try std.testing.expectEqual(@as(f64, 0.0), up.value.beta);
    const right = m.outputs(.{}, .{ .velocity_body = .{ .x = 1.0, .y = 1.0 } }, .{});
    try std.testing.expect(right.value.beta > 0.0);
    try std.testing.expectEqual(@as(f64, 0.0), right.value.alpha);
}

test "the body rate shifts the flow at an offset station" {
    const m = station();
    // omega = +z, r = +x: omega x r = +y, so the station sees 1 m/s to the right.
    const r = m.outputs(
        .{},
        .{ .rate_body = .{ .z = 1.0 } },
        placement(.{ .x = 1.0 }, 0.0),
    );
    try std.testing.expectApproxEqAbs(@as(f64, 1.0), r.value.airspeed, tolerance);
    try std.testing.expectApproxEqAbs(std.math.pi / 2.0, r.value.beta, tolerance);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), r.value.alpha, tolerance);
}

test "a rotated placement turns the body velocity into the local frame" {
    const m = station();
    // Local +x is body +y, so body velocity along +y is pure local +x flow.
    const r = m.outputs(
        .{},
        .{ .velocity_body = .{ .y = 10.0 } },
        placement(.{}, std.math.pi / 2.0),
    );
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), r.value.alpha, tolerance);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), r.value.beta, tolerance);
    try std.testing.expectApproxEqAbs(@as(f64, 10.0), r.value.airspeed, tolerance);
}

test "zero flow is valid and gives zero angles" {
    const r = station().outputs(.{}, .{}, placement(.{ .x = 1.0 }, 0.5));
    try std.testing.expectEqual(base.Validity.valid, r.validity);
    try std.testing.expectEqual(@as(f64, 0.0), r.value.alpha);
    try std.testing.expectEqual(@as(f64, 0.0), r.value.beta);
    try std.testing.expectEqual(@as(f64, 0.0), r.value.airspeed);
}

test "airflow honors the component contract" {
    contract.assertKind(Model);
    const m = station();
    var rng = base.Rng.init(51);
    try contract.expectKindContract(Model, &m, &rng, 2000);
}
