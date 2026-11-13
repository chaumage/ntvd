//! Gain: y = k * x on dimensionless signals.
//!
//! The smallest useful component: efficiencies, load factors, safety margins and unit-free
//! corrections are all gains. It is also the simplest kind with feedthrough, which makes it
//! the natural building block for testing how the model compiler handles loops.
//!
//! Models
//!   linear   y = k * x

const base = @import("base");
const contract = @import("contract.zig");

pub const Model = union(enum) {
    linear: Linear,

    pub const In = struct { x: f64 };
    pub const Out = struct { y: f64 };
    pub const State = struct {};

    pub const signature = contract.signature(Model, .{
        .in_dims = .{ .x = base.dim.dimensionless },
        .out_dims = .{ .y = base.dim.dimensionless },
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

pub const Linear = struct {
    k: f64,

    pub fn validate(self: *const Linear) contract.ParamCheck {
        if (!contract.isFinite(self.k)) return "k must be finite";
        return null;
    }

    pub fn outputs(self: *const Linear, x: Model.State, u: Model.In) contract.Evaluated(Model.Out) {
        _ = x;
        contract.assertFiniteFields(u);
        return .{ .value = .{ .y = self.k * u.x } };
    }
};

test "linear gain multiplies" {
    const m: Model = .{ .linear = .{ .k = -2.5 } };
    const r = m.outputs(.{}, .{ .x = 4.0 });
    try @import("std").testing.expectEqual(@as(f64, -10.0), r.value.y);
    try @import("std").testing.expectEqual(base.Validity.valid, r.validity);
}

test "gain parameters are validated" {
    const std = @import("std");
    const ok: Model = .{ .linear = .{ .k = 0.0 } };
    try std.testing.expectEqual(@as(contract.ParamCheck, null), ok.validate());
    const bad: Model = .{ .linear = .{ .k = std.math.nan(f64) } };
    try std.testing.expect(bad.validate() != null);
}

test "gain honors the component contract" {
    contract.assertKind(Model);
    const m: Model = .{ .linear = .{ .k = 3.0 } };
    var rng = base.Rng.init(11);
    try contract.expectKindContract(Model, &m, &rng, 500);
}
