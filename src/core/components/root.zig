//! Layer 1: physical components. Depends on `base` only.
//!
//! The contract every component honors, and the closed set of component kinds. Each kind
//! owns the physics of one role and offers several models of increasing fidelity.

pub const contract = @import("contract.zig");
pub const registry = @import("registry.zig");

pub const Component = registry.Component;

pub const air = registry.air;
pub const body = registry.body;
pub const frame_transform = registry.frame_transform;
pub const gain = registry.gain;
pub const lift_surface = registry.lift_surface;
pub const load = registry.load;
pub const point_mass = registry.point_mass;
pub const thruster = registry.thruster;

test {
    _ = @import("contract.zig");
    _ = @import("registry.zig");
    _ = @import("air.zig");
    _ = @import("body.zig");
    _ = @import("frame_transform.zig");
    _ = @import("gain.zig");
    _ = @import("lift_surface.zig");
    _ = @import("load.zig");
    _ = @import("point_mass.zig");
    _ = @import("thruster.zig");
}
