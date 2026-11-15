//! Frame transform: expresses a vector or a wrench in another frame.
//!
//! Instances are created by `compile`, never authored, and carry the whole transform in the
//! model value: a pre-rotation, an optional body-attitude rotation, a post-rotation and an
//! offset. One kind serves every frame change the compiler needs, so evaluation stays a plain
//! node in the graph.
//!
//! Modes
//!   rotate   v_out = q_eff * v_in
//!   mount    F_body = q_eff * F_local,  M_body = q_eff * M_local + r x F_body
//!
//! `q_eff = pre * q? * post`, where `q` is the body attitude input. `dynamic` selects whether
//! the body attitude enters directly (body -> parent), through its conjugate (parent -> body),
//! or not at all. Every port is summing: a node uses only the input its mode needs.

const std = @import("std");
const base = @import("base");
const contract = @import("contract.zig");

pub const Mode = enum { rotate, mount };

pub const Dynamic = enum { none, direct, conjugate };

/// The transform an instance applies. All fields are resolved at compile time.
pub const Apply = struct {
    mode: Mode,
    /// Rotation applied after the dynamic body attitude (or alone when static).
    pre: base.Quat = .{},
    /// Rotation applied last, in the destination frame.
    post: base.Quat = .{},
    dynamic: Dynamic = .none,
    /// Position of the source origin in body coordinates; used by `mount`.
    r: base.Vec3 = .{},

    pub fn validate(self: *const Apply) contract.ParamCheck {
        if (!self.pre.isFinite() or !self.post.isFinite()) return "rotation must be finite";
        if (base.Quat.normalized(self.pre) == null) return "pre-rotation must be non-zero";
        if (base.Quat.normalized(self.post) == null) return "post-rotation must be non-zero";
        if (!self.r.isFinite()) return "offset must be finite";
        return null;
    }

    pub fn outputs(
        self: *const Apply,
        x: Model.State,
        u: Model.In,
        ctx: contract.Context,
    ) contract.Evaluated(Model.Out) {
        _ = x;
        _ = ctx;
        contract.assertFiniteFields(u);
        const q = self.effective(u.q);
        return switch (self.mode) {
            .rotate => .{ .value = .{ .v = q.rotate(u.v) } },
            .mount => blk: {
                const force = q.rotate(u.wrench.force);
                const moment = q.rotate(u.wrench.moment).add(self.r.cross(force));
                break :blk .{ .value = .{ .wrench = .{ .force = force, .moment = moment } } };
            },
        };
    }

    fn effective(self: *const Apply, q: base.Quat) base.Quat {
        const mid = switch (self.dynamic) {
            .none => base.Quat.identity,
            .direct => q,
            .conjugate => base.Quat.conjugate(q),
        };
        return self.pre.mul(mid).mul(self.post);
    }
};

pub const Model = union(enum) {
    apply: Apply,

    pub const In = struct {
        v: base.Vec3 = .{},
        q: base.Quat = .{},
        omega: base.Vec3 = .{},
        wrench: base.Wrench = .{},
    };
    pub const Out = struct {
        v: base.Vec3 = .{},
        wrench: base.Wrench = .{},
    };
    pub const State = struct {};

    pub const signature = contract.signature(Model, .{
        .in = .{
            .v = contract.vec3(base.dim.length, .scalar, .local),
            .q = contract.quat(.body),
            .omega = contract.vec3(base.dim.angular_rate, .angular_rate, .body),
            .wrench = contract.wrench(.local),
        },
        .in_sum = .{ .v = true, .q = true, .omega = true, .wrench = true },
        .out = .{
            .v = contract.vec3(base.dim.length, .scalar, .local),
            .wrench = contract.wrench(.local),
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

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

fn expectVec(expected: base.Vec3, actual: base.Vec3) !void {
    try std.testing.expectApproxEqAbs(expected.x, actual.x, 1.0e-12);
    try std.testing.expectApproxEqAbs(expected.y, actual.y, 1.0e-12);
    try std.testing.expectApproxEqAbs(expected.z, actual.z, 1.0e-12);
}

test "rotate applies the pre-rotation to a vector" {
    const m: Model = .{ .apply = .{
        .mode = .rotate,
        .pre = base.Quat.fromEuler(0.0, 0.0, std.math.pi / 2.0),
    } };
    const r = m.outputs(.{}, .{ .v = .{ .x = 1.0 } }, .{});
    try expectVec(.{ .y = 1.0 }, r.value.v);
    try std.testing.expectEqual(base.Validity.valid, r.validity);
}

test "a dynamic conjugate maps world back to body" {
    const yaw = base.Quat.fromEuler(0.0, 0.0, std.math.pi / 2.0);
    const direct: Model = .{ .apply = .{ .mode = .rotate, .dynamic = .direct } };
    const inverse: Model = .{ .apply = .{ .mode = .rotate, .dynamic = .conjugate } };
    const v = base.Vec3{ .x = 1.0 };
    try expectVec(.{ .y = 1.0 }, direct.outputs(.{}, .{ .v = v, .q = yaw }, .{}).value.v);
    try expectVec(.{ .y = -1.0 }, inverse.outputs(.{}, .{ .v = v, .q = yaw }, .{}).value.v);
}

test "mount rotates the force and translates the moment by r cross F" {
    // A local +x force under a 90 degree yaw becomes body +y. At r = (0, 0, -0.4),
    // r x F = (0.8, 0, 0): a force above the origin pitches the body.
    const m: Model = .{ .apply = .{
        .mode = .mount,
        .pre = base.Quat.fromEuler(0.0, 0.0, std.math.pi / 2.0),
        .r = .{ .z = -0.4 },
    } };
    const r = m.outputs(.{}, .{ .wrench = .{ .force = .{ .x = 2.0 } } }, .{});
    try expectVec(.{ .y = 2.0 }, r.value.wrench.force);
    try expectVec(.{ .x = 0.8 }, r.value.wrench.moment);
}

test "transform parameters are validated" {
    const ok: Apply = .{ .mode = .rotate };
    try std.testing.expectEqual(@as(contract.ParamCheck, null), ok.validate());
    const zero_q: Apply = .{ .mode = .rotate, .pre = .{ .w = 0.0 } };
    try std.testing.expect(zero_q.validate() != null);
    const bad_offset: Apply = .{ .mode = .mount, .r = .{ .x = std.math.nan(f64) } };
    try std.testing.expect(bad_offset.validate() != null);
}

test "frame transform honors the component contract" {
    contract.assertKind(Model);
    var rng = base.Rng.init(61);
    const models = [_]Model{
        .{ .apply = .{ .mode = .rotate, .dynamic = .direct } },
        .{ .apply = .{
            .mode = .mount,
            .pre = base.Quat.fromEuler(0.1, -0.2, 0.3),
            .r = .{ .x = 0.3, .z = -0.4 },
        } },
    };
    for (&models) |*m| try contract.expectKindContract(Model, m, &rng, 500);
}
