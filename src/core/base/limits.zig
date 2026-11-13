//! Hard capacity limits of the whole core.
//!
//! Every container in the core is statically sized from these constants. Nothing allocates
//! after startup, so a model that does not fit is rejected up front with a precise error
//! instead of failing somewhere deep in a solver loop.

const std = @import("std");

/// Components (wing, propeller, hull, ...) in one vehicle architecture.
pub const max_components: u32 = 64;

/// Boundary signals driven from outside the model (operating conditions, trim variables).
pub const max_externals: u32 = 32;

/// Inputs, outputs or states of one single component.
pub const max_ports: u32 = 8;

/// Connections between ports, after expansion. A summing input counts one link per source.
pub const max_links: u32 = 256;

/// Longest name of a component or external signal, in bytes.
pub const max_name_len: u32 = 32;

/// Points of one measured curve or one axis of a measured map.
pub const max_table_points: u32 = 32;

/// Unknowns of one nonlinear system handled by the dense solvers.
pub const max_unknowns: u32 = 32;

/// Signals of a compiled model: every external plus every component output.
pub const max_signals: u32 = max_externals + max_components * max_ports;

/// Continuous states of a compiled model.
pub const max_states: u32 = max_components * max_ports;

comptime {
    // A compiled plan stores signal and link indices as u16.
    std.debug.assert(max_signals <= std.math.maxInt(u16));
    std.debug.assert(max_links <= std.math.maxInt(u16));
    std.debug.assert(max_states <= std.math.maxInt(u16));
    std.debug.assert(max_name_len <= std.math.maxInt(u8));
    std.debug.assert(max_ports <= std.math.maxInt(u8));
    // A component must be able to hold its own ports in the scratch buffers of the plan.
    std.debug.assert(max_ports >= 1 and max_table_points >= 2);
}

test "limits are mutually consistent" {
    try std.testing.expect(max_signals >= max_externals + max_components);
    try std.testing.expect(max_states >= max_components);
    try std.testing.expect(max_unknowns >= 1);
}
