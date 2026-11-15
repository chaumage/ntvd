//! The closed set of component kinds, as one tagged union.
//!
//! A tagged union gives exhaustive `switch` checking: adding a kind here makes the compiler
//! point at every place that must learn about it. There are no vtables and no allocation,
//! and dispatch stays visible in the source. Every kind is checked against the component
//! contract at compile time, so a malformed kind cannot even be built.
//!
//! Selecting a model is selecting a variant of the kind's own union:
//!   `.{ .lift_surface = .{ .linear = .{ ... } } }`
//!   `.{ .lift_surface = .{ .polar_table = .{ ... } } }`
//! Both have the same signature, so one can replace the other in an existing architecture.

const std = @import("std");
const base = @import("base");
const contract = @import("contract.zig");

pub const air = @import("air.zig");
pub const body = @import("body.zig");
pub const frame_transform = @import("frame_transform.zig");
pub const gain = @import("gain.zig");
pub const lift_surface = @import("lift_surface.zig");
pub const load = @import("load.zig");
pub const point_mass = @import("point_mass.zig");
pub const thruster = @import("thruster.zig");

pub const Component = union(enum) {
    air: air.Model,
    body: body.Model,
    frame_transform: frame_transform.Model,
    gain: gain.Model,
    lift_surface: lift_surface.Model,
    load: load.Model,
    point_mass: point_mass.Model,
    thruster: thruster.Model,

    pub fn signature(self: *const Component) *const contract.Signature {
        return switch (self.*) {
            inline else => |*m| &@TypeOf(m.*).signature,
        };
    }

    /// Parameter check, null when usable.
    pub fn validate(self: *const Component) contract.ParamCheck {
        return switch (self.*) {
            inline else => |*m| m.validate(),
        };
    }

    pub fn outputs(
        self: *const Component,
        x: []const f64,
        u: []const f64,
        y: []f64,
        ctx: contract.Context,
    ) base.Validity {
        return switch (self.*) {
            inline else => |*m| contract.outputsFlat(@TypeOf(m.*), m, x, u, y, ctx),
        };
    }

    pub fn derivatives(
        self: *const Component,
        x: []const f64,
        u: []const f64,
        dx: []f64,
        ctx: contract.Context,
    ) base.Validity {
        return switch (self.*) {
            inline else => |*m| contract.derivativesFlat(@TypeOf(m.*), m, x, u, dx, ctx),
        };
    }

    pub fn kindName(self: *const Component) []const u8 {
        return @tagName(self.*);
    }

    pub fn modelName(self: *const Component) []const u8 {
        return switch (self.*) {
            inline else => |*m| @tagName(m.*),
        };
    }
};

comptime {
    for (@typeInfo(Component).@"union".field_types) |kind| contract.assertKind(kind);
    // Components are stored by value in the compiled plan: guard against accidental bloat.
    std.debug.assert(@sizeOf(Component) <= 2048);
}

test "every kind is registered with a distinct name" {
    const names = @typeInfo(Component).@"union".field_names;
    for (names, 0..) |a, i| {
        for (names[i + 1 ..]) |b| try std.testing.expect(!std.mem.eql(u8, a, b));
    }
}

test "signature and names are reachable through the union" {
    const c: Component = .{ .lift_surface = .{ .linear = .{
        .area_m2 = 1.0,
        .aspect_ratio = 6.0,
        .cl_alpha_per_rad = 5.0,
        .cd0 = 0.02,
        .oswald = 0.85,
        .alpha_stall_rad = 0.3,
    } } };
    const sig = c.signature();
    try std.testing.expectEqual(@as(?u32, 1), sig.inputIndex("alpha"));
    try std.testing.expectEqual(@as(?u32, 1), sig.outputIndex("drag"));
    try std.testing.expectEqualStrings("lift_surface", c.kindName());
    try std.testing.expectEqualStrings("linear", c.modelName());
    try std.testing.expect(c.validate() == null);
}

test "flat evaluation through the union matches the typed model" {
    const model: thruster.Model = .{ .linear_decay = .{
        .static_thrust_n = 40.0,
        .max_speed_ms = 60.0,
    } };
    const c: Component = .{ .thruster = model };
    var y: [1]f64 = undefined;
    const v = c.outputs(&.{}, &.{ 12.0, 0.5 }, &y, .{});
    const typed = model.outputs(.{}, .{ .airspeed = 12.0, .throttle = 0.5 }, .{});
    try std.testing.expectEqual(typed.value.thrust, y[0]);
    try std.testing.expectEqual(typed.validity, v);
}

test "stateless kinds have nothing to differentiate" {
    const c: Component = .{ .gain = .{ .linear = .{ .k = 2.0 } } };
    var none: [0]f64 = .{};
    try std.testing.expectEqual(base.Validity.valid, c.derivatives(&.{}, &.{1.0}, &none, .{}));
}

test "a dynamic kind exposes derivatives through the union" {
    const c: Component = .{ .point_mass = .{ .rigid = .{ .mass_kg = 4.0 } } };
    var dx: [2]f64 = undefined;
    _ = c.derivatives(&.{ 2.0, 0.0 }, &.{8.0}, &dx, .{});
    try std.testing.expectEqual(@as(f64, 2.0), dx[0]);
    try std.testing.expectEqual(@as(f64, 2.0), dx[1]);
}
