//! Body: the vehicle frame. A static attitude holder and the summing mount point where
//! rigidly attached components will deliver their wrench.
//!
//! Input   mount [N, N*m] in body coordinates (summing); read by the dynamics, not by `outputs`
//! Output  attitude (body -> world)
//! State   attitude (body -> world), as a unit quaternion
//!
//! Phase 3 has no body rate, so the attitude derivative is zero. The kind is deliberately
//! `feedthrough = false`: `attitude` comes from the state alone, which lets world-frame
//! transforms read it without creating an algebraic loop through the mount point. Phase 4
//! replaces this kind with a six-degree-of-freedom rigid body that integrates the same `mount`
//! input.
//!
//! The compiler identifies the designated body by the presence of the wrench `mount` input; a
//! plan with more than one such component is ambiguous.

const std = @import("std");
const base = @import("base");
const contract = @import("contract.zig");

pub const Model = union(enum) {
    static: Static,

    pub const In = struct { mount: base.Wrench = .{} };
    pub const Out = struct { attitude: base.Quat };
    pub const State = struct { attitude: base.Quat };

    pub const signature = contract.signature(Model, .{
        .in = .{ .mount = contract.wrench(.body) },
        .in_sum = .{ .mount = true },
        .out = .{ .attitude = contract.quat(.world) },
        .state = .{ .attitude = contract.quat(.world) },
        .feedthrough = false,
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

    pub fn derivatives(
        self: *const Model,
        x: State,
        u: In,
        ctx: contract.Context,
    ) contract.Evaluated(State) {
        return switch (self.*) {
            inline else => |*m| m.derivatives(x, u, ctx),
        };
    }
};

pub const Static = struct {
    pub fn validate(self: *const Static) contract.ParamCheck {
        _ = self;
        return null;
    }

    /// Does not read the input: this is what `feedthrough = false` promises.
    pub fn outputs(
        self: *const Static,
        x: Model.State,
        u: Model.In,
        ctx: contract.Context,
    ) contract.Evaluated(Model.Out) {
        _ = self;
        _ = u;
        _ = ctx;
        return .{ .value = .{ .attitude = x.attitude } };
    }

    /// No rate state in Phase 3, so the attitude does not change. The mount input is reserved
    /// for the Phase 4 rigid body, which integrates it.
    pub fn derivatives(
        self: *const Static,
        x: Model.State,
        u: Model.In,
        ctx: contract.Context,
    ) contract.Evaluated(Model.State) {
        _ = self;
        _ = u;
        _ = ctx;
        return .{ .value = .{ .attitude = base.Quat.derivative(x.attitude, base.Vec3.zero) } };
    }
};

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

test "the attitude output is the state and ignores the mount" {
    const m: Model = .{ .static = .{} };
    const q = base.Quat.fromEuler(0.2, -0.1, 0.5);
    const r = m.outputs(
        .{ .attitude = q },
        .{ .mount = .{ .force = .{ .x = 1.0 } } },
        .{},
    );
    try std.testing.expectEqual(q, r.value.attitude);
    try std.testing.expectEqual(base.Validity.valid, r.validity);
}

test "the attitude derivative is zero in phase 3" {
    const m: Model = .{ .static = .{} };
    const q = base.Quat.fromEuler(0.2, -0.1, 0.5);
    const d = m.derivatives(.{ .attitude = q }, .{}, .{});
    try std.testing.expectEqual(@as(f64, 0.0), d.value.attitude.w);
    try std.testing.expectEqual(@as(f64, 0.0), d.value.attitude.x);
    try std.testing.expectEqual(@as(f64, 0.0), d.value.attitude.y);
    try std.testing.expectEqual(@as(f64, 0.0), d.value.attitude.z);
}

test "body honors the component contract" {
    contract.assertKind(Model);
    const m: Model = .{ .static = .{} };
    var rng = base.Rng.init(71);
    try contract.expectKindContract(Model, &m, &rng, 500);
}

test "the mount input sums and the attitude is a quaternion state" {
    const sig = Model.signature;
    try std.testing.expectEqual(contract.FanIn.sum, sig.inputs[0].ptype.fan_in);
    try std.testing.expectEqual(@as(u16, 6), sig.inputSlots());
    try std.testing.expectEqual(@as(u16, 4), sig.outputSlots());
    try std.testing.expectEqual(@as(u16, 4), sig.stateSlots());
    try std.testing.expectEqual(contract.Quantity.quaternion, sig.states[0].ptype.quantity);
    try std.testing.expect(!sig.feedthrough);
}
