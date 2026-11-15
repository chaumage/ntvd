//! Load: a force and a moment applied at one point of a component.
//!
//! It is the authoring aid for rigid attachment: express the local force and moment, attach the
//! component once, and the compiler mounts the wrench on the body. The echo outputs let a
//! local-frame force or moment be wired to another local port without a hand-written rotate.
//!
//! Inputs   force [N], moment [N*m] in the component's local frame (summing)
//! Output   wrench (local), plus force and moment echoes (local)
//!
//! Stateless and feedthrough: it only groups its inputs.

const std = @import("std");
const base = @import("base");
const contract = @import("contract.zig");

pub const Model = union(enum) {
    point: Point,

    pub const In = struct {
        force: base.Vec3 = .{},
        moment: base.Vec3 = .{},
    };
    pub const Out = struct {
        wrench: base.Wrench,
        force: base.Vec3,
        moment: base.Vec3,
    };
    pub const State = struct {};

    pub const signature = contract.signature(Model, .{
        .in = .{
            .force = contract.vec3(base.dim.force, .force, .local),
            .moment = contract.vec3(base.dim.moment, .moment, .local),
        },
        .in_sum = .{ .force = true, .moment = true },
        .out = .{
            .wrench = contract.wrench(.local),
            .force = contract.vec3(base.dim.force, .force, .local),
            .moment = contract.vec3(base.dim.moment, .moment, .local),
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

pub const Point = struct {
    pub fn validate(self: *const Point) contract.ParamCheck {
        _ = self;
        return null;
    }

    pub fn outputs(
        self: *const Point,
        x: Model.State,
        u: Model.In,
        ctx: contract.Context,
    ) contract.Evaluated(Model.Out) {
        _ = self;
        _ = x;
        _ = ctx;
        contract.assertFiniteFields(u);
        return .{ .value = .{
            .wrench = .{ .force = u.force, .moment = u.moment },
            .force = u.force,
            .moment = u.moment,
        } };
    }
};

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

test "load groups a force and a moment into a local wrench" {
    const m: Model = .{ .point = .{} };
    const u: Model.In = .{
        .force = .{ .x = 4.0, .y = -1.0 },
        .moment = .{ .z = 2.5 },
    };
    const r = m.outputs(.{}, u, .{});
    try std.testing.expectEqual(@as(f64, 4.0), r.value.wrench.force.x);
    try std.testing.expectEqual(@as(f64, -1.0), r.value.wrench.force.y);
    try std.testing.expectEqual(@as(f64, 2.5), r.value.wrench.moment.z);
    try std.testing.expectEqual(u.force, r.value.force);
    try std.testing.expectEqual(u.moment, r.value.moment);
    try std.testing.expectEqual(base.Validity.valid, r.validity);
}

test "load ignores its placement" {
    const m: Model = .{ .point = .{} };
    const u: Model.In = .{ .force = .{ .z = 1.0 } };
    const placement: contract.Context = .{ .placement = .{
        .position = .{ .x = 3.0 },
        .orientation = base.Quat.fromEuler(0.1, 0.2, 0.3),
    } };
    try std.testing.expectEqual(m.outputs(.{}, u, .{}).value, m.outputs(.{}, u, placement).value);
}

test "load honors the component contract" {
    contract.assertKind(Model);
    const m: Model = .{ .point = .{} };
    var rng = base.Rng.init(81);
    try contract.expectKindContract(Model, &m, &rng, 500);
}
