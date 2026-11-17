//! Layer 3: numerical methods acting on a compiled plan. Depends on `base` and `model`.
//!
//! Today: fixed-step time integration and damped Newton on small residual systems.

pub const integrate = @import("integrate.zig");
pub const newton = @import("newton.zig");

test {
    _ = @import("integrate.zig");
    _ = @import("newton.zig");
}
