//! Representation and policy limits.
//!
//! Plan and workspace sizes are not fixed here: they are computed from the spec and carved
//! from caller-provided storage (`model.storage`). What remains are the widths of the encoding
//! and a few solver policies. A model that exceeds an encoding ceiling is rejected up front
//! with a precise error instead of overflowing an index.

const std = @import("std");

/// Inputs, outputs or states of one single component. Stored as a `u8` count per node.
pub const max_ports: u32 = 16;

/// Slots of one wide port (a wrench is force + moment = 6).
pub const max_slots_per_port: u32 = 6;

/// Input, output or state slots of one single component, the widest every port can be.
pub const max_slots: u32 = max_ports * max_slots_per_port;

/// Longest name of a component or external signal, in bytes. Generated transform node names
/// (`__x_<n>`) are subject to the same limit.
pub const max_name_len: u32 = 32;

/// Points of one measured curve or one axis of a measured map.
pub const max_table_points: u32 = 32;

/// Unknowns of one nonlinear system handled by the dense solvers.
pub const max_unknowns: u32 = 32;

/// Counts and indices in a compiled plan are stored as `u16`.
pub const max_index: u32 = std.math.maxInt(u16);

comptime {
    std.debug.assert(max_index == std.math.maxInt(u16));
    std.debug.assert(max_name_len <= std.math.maxInt(u8));
    std.debug.assert(max_ports <= std.math.maxInt(u8));
    std.debug.assert(max_slots_per_port <= std.math.maxInt(u8));
    std.debug.assert(max_slots == max_ports * max_slots_per_port);
    std.debug.assert(max_slots >= 1 and max_ports >= 1 and max_table_points >= 2);
}

test "limits are mutually consistent" {
    try std.testing.expect(max_index >= max_ports);
    try std.testing.expect(max_index >= max_slots_per_port);
    try std.testing.expect(max_slots >= max_slots_per_port);
    try std.testing.expect(max_unknowns >= 1);
}
