//! Layer 0: foundations. Depends on nothing but `std`.
//!
//! Dimensions, units, validity status, fixed-capacity containers, deterministic random
//! numbers, small dense linear algebra, lookup tables and reference fluid properties.

pub const limits = @import("limits.zig");
pub const units = @import("units.zig");
pub const dim = @import("dim.zig");
pub const geometry = @import("geometry.zig");
pub const interp = @import("interp.zig");
pub const linalg = @import("linalg.zig");
pub const atmosphere = @import("atmosphere.zig");

pub const Dim = dim.Dim;
pub const Validity = @import("validity.zig").Validity;
pub const StaticVec = @import("static_vec.zig").StaticVec;
pub const Name = @import("name.zig").Name;
pub const Rng = @import("rng.zig").Rng;
pub const Vec3 = geometry.Vec3;
pub const Quat = geometry.Quat;
pub const Wrench = geometry.Wrench;
pub const Rotation = geometry.Rotation;
pub const Placement = geometry.Placement;

test {
    _ = @import("limits.zig");
    _ = @import("units.zig");
    _ = @import("dim.zig");
    _ = @import("geometry.zig");
    _ = @import("interp.zig");
    _ = @import("linalg.zig");
    _ = @import("atmosphere.zig");
    _ = @import("validity.zig");
    _ = @import("static_vec.zig");
    _ = @import("name.zig");
    _ = @import("rng.zig");
}
