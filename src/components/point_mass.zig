//! Point mass: the first dynamic component. A rigid body translating along one axis.
//!
//! Input    force [N]  (summing port: every connected force is added, with its own sign)
//! Outputs  velocity [m/s], position [m]
//! States   velocity, position
//!
//!   d(velocity)/dt = force / mass          d(position)/dt = velocity
//!
//! The outputs are the states themselves, so the component has no feedthrough: it is what
//! closes physical loops (velocity -> drag -> force -> acceleration -> velocity) without
//! an algebraic solver. The same pattern, with more states, is the base of full 3D or 6-DOF
//! rigid bodies and of battery, motor or actuator dynamics.
//!
//! Models
//!   rigid   constant mass

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const contract = @import("contract.zig");

pub const Model = union(enum) {
    rigid: Rigid,

    pub const In = struct { force: f64 };
    pub const Out = struct { velocity: f64, position: f64 };
    pub const State = struct { velocity: f64, position: f64 };

    pub const signature = contract.signature(Model, .{
        .in_dims = .{ .force = base.dim.force },
        .in_sum = .{ .force = true },
        .out_dims = .{ .velocity = base.dim.speed, .position = base.dim.length },
        .state_dims = .{ .velocity = base.dim.speed, .position = base.dim.length },
        .feedthrough = false,
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

    pub fn derivatives(self: *const Model, x: State, u: In) contract.Evaluated(State) {
        return switch (self.*) {
            inline else => |*m| m.derivatives(x, u),
        };
    }
};

pub const Rigid = struct {
    mass_kg: f64,

    pub fn validate(self: *const Rigid) contract.ParamCheck {
        if (!contract.isPositive(self.mass_kg)) return "mass_kg must be positive";
        return null;
    }

    /// Does not read the input: this is what `feedthrough = false` promises.
    pub fn outputs(self: *const Rigid, x: Model.State, u: Model.In) contract.Evaluated(Model.Out) {
        _ = self;
        _ = u;
        return .{ .value = .{ .velocity = x.velocity, .position = x.position } };
    }

    pub fn derivatives(
        self: *const Rigid,
        x: Model.State,
        u: Model.In,
    ) contract.Evaluated(Model.State) {
        contract.assertFiniteFields(x);
        contract.assertFiniteFields(u);
        assert(self.mass_kg > 0.0);
        return .{ .value = .{
            .velocity = u.force / self.mass_kg,
            .position = x.velocity,
        } };
    }
};

test "newton's second law" {
    const m: Model = .{ .rigid = .{ .mass_kg = 10.0 } };
    const r = m.derivatives(.{ .velocity = 3.0, .position = 7.0 }, .{ .force = 25.0 });
    try std.testing.expectEqual(@as(f64, 2.5), r.value.velocity);
    try std.testing.expectEqual(@as(f64, 3.0), r.value.position);
    try std.testing.expectEqual(base.Validity.valid, r.validity);
}

test "outputs are the states and ignore the input" {
    const m: Model = .{ .rigid = .{ .mass_kg = 10.0 } };
    const r = m.outputs(.{ .velocity = 3.0, .position = 7.0 }, .{ .force = std.math.nan(f64) });
    try std.testing.expectEqual(@as(f64, 3.0), r.value.velocity);
    try std.testing.expectEqual(@as(f64, 7.0), r.value.position);
}

test "the force port sums, the others do not" {
    try std.testing.expectEqual(contract.FanIn.sum, Model.signature.inputs[0].fan_in);
    try std.testing.expect(!Model.signature.feedthrough);
    try std.testing.expectEqual(@as(usize, 2), Model.signature.states.len);
}

test "point mass parameters are validated" {
    const ok: Model = .{ .rigid = .{ .mass_kg = 0.5 } };
    try std.testing.expectEqual(@as(contract.ParamCheck, null), ok.validate());
    const zero: Model = .{ .rigid = .{ .mass_kg = 0.0 } };
    try std.testing.expect(zero.validate() != null);
    const inf: Model = .{ .rigid = .{ .mass_kg = std.math.inf(f64) } };
    try std.testing.expect(inf.validate() != null);
}

test "point mass honors the component contract" {
    contract.assertKind(Model);
    const m: Model = .{ .rigid = .{ .mass_kg = 12.0 } };
    var rng = base.Rng.init(51);
    try contract.expectKindContract(Model, &m, &rng, 2000);
}
