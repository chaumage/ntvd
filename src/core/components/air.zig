//! Air: the medium a vehicle flies in. Altitude in, density and speed of sound out.
//!
//! Models (increasing fidelity is added here, nothing else changes)
//!   isa     International Standard Atmosphere, optionally ISA+delta_t
//!   fixed   constant density and speed of sound, e.g. a wind-tunnel condition or a
//!           measured day. Useful to freeze the environment during an optimization.

const std = @import("std");
const base = @import("base");
const contract = @import("contract.zig");

pub const Model = union(enum) {
    isa: Isa,
    fixed: Fixed,

    pub const In = struct { altitude: f64 };
    pub const Out = struct { density: f64, speed_of_sound: f64 };
    pub const State = struct {};

    pub const signature = contract.signature(Model, .{
        .in = .{ .altitude = contract.scalar(base.dim.length) },
        .out = .{
            .density = contract.scalar(base.dim.density),
            .speed_of_sound = contract.scalar(base.dim.speed),
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

pub const Isa = struct {
    /// Temperature offset from the standard day, in kelvin.
    delta_t_k: f64 = 0.0,

    pub fn validate(self: *const Isa) contract.ParamCheck {
        if (!contract.isFinite(self.delta_t_k)) return "delta_t_k must be finite";
        if (self.delta_t_k < -60.0 or self.delta_t_k > 60.0) {
            return "delta_t_k must be within +-60 K of the standard day";
        }
        return null;
    }

    pub fn outputs(
        self: *const Isa,
        x: Model.State,
        u: Model.In,
        ctx: contract.Context,
    ) contract.Evaluated(Model.Out) {
        _ = x;
        _ = ctx;
        contract.assertFiniteFields(u);
        const r = base.atmosphere.isa(u.altitude, self.delta_t_k);
        return .{
            .value = .{ .density = r.air.density, .speed_of_sound = r.air.speed_of_sound },
            .validity = r.validity,
        };
    }
};

pub const Fixed = struct {
    density_kg_m3: f64,
    speed_of_sound_ms: f64,

    pub fn validate(self: *const Fixed) contract.ParamCheck {
        if (!contract.isPositive(self.density_kg_m3)) return "density_kg_m3 must be positive";
        if (!contract.isPositive(self.speed_of_sound_ms)) {
            return "speed_of_sound_ms must be positive";
        }
        return null;
    }

    pub fn outputs(
        self: *const Fixed,
        x: Model.State,
        u: Model.In,
        ctx: contract.Context,
    ) contract.Evaluated(Model.Out) {
        _ = x;
        _ = ctx;
        contract.assertFiniteFields(u);
        return .{ .value = .{
            .density = self.density_kg_m3,
            .speed_of_sound = self.speed_of_sound_ms,
        } };
    }
};

test "isa model reproduces the standard atmosphere" {
    const m: Model = .{ .isa = .{} };
    const r = m.outputs(.{}, .{ .altitude = 0.0 }, .{});
    try std.testing.expectApproxEqRel(@as(f64, 1.225), r.value.density, 1e-4);
    try std.testing.expectApproxEqRel(@as(f64, 340.294), r.value.speed_of_sound, 1e-5);
    try std.testing.expectEqual(base.Validity.valid, r.validity);
}

test "isa flags altitudes beyond its range" {
    const m: Model = .{ .isa = .{} };
    const r = m.outputs(.{}, .{ .altitude = 80_000.0 }, .{});
    try std.testing.expectEqual(base.Validity.beyond_envelope, r.validity);
    try std.testing.expect(std.math.isFinite(r.value.density));
}

test "a hot day is less dense" {
    const std_day: Model = .{ .isa = .{} };
    const hot: Model = .{ .isa = .{ .delta_t_k = 20.0 } };
    const u: Model.In = .{ .altitude = 2000.0 };
    const cool_density = std_day.outputs(.{}, u, .{}).value.density;
    try std.testing.expect(hot.outputs(.{}, u, .{}).value.density < cool_density);
}

test "fixed model ignores altitude" {
    const m: Model = .{ .fixed = .{ .density_kg_m3 = 1.1, .speed_of_sound_ms = 330.0 } };
    const a = m.outputs(.{}, .{ .altitude = 0.0 }, .{});
    const b = m.outputs(.{}, .{ .altitude = 9000.0 }, .{});
    try std.testing.expectEqual(a.value.density, b.value.density);
    try std.testing.expectEqual(@as(f64, 330.0), b.value.speed_of_sound);
    try std.testing.expectEqual(base.Validity.valid, b.validity);
}

test "air parameters are validated" {
    const ok: Model = .{ .isa = .{ .delta_t_k = 15.0 } };
    try std.testing.expectEqual(@as(contract.ParamCheck, null), ok.validate());
    const too_hot: Model = .{ .isa = .{ .delta_t_k = 500.0 } };
    try std.testing.expect(too_hot.validate() != null);
    const vacuum: Model = .{ .fixed = .{ .density_kg_m3 = 0.0, .speed_of_sound_ms = 300.0 } };
    try std.testing.expect(vacuum.validate() != null);
}

test "every air model honors the component contract" {
    contract.assertKind(Model);
    var rng = base.Rng.init(21);
    const models = [_]Model{
        .{ .isa = .{} },
        .{ .isa = .{ .delta_t_k = -30.0 } },
        .{ .fixed = .{ .density_kg_m3 = 1.0, .speed_of_sound_ms = 340.0 } },
    };
    for (&models) |*m| try contract.expectKindContract(Model, m, &rng, 500);
}
