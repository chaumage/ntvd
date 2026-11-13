//! Layer 3: numerical methods acting on a compiled plan. Depends on `base` and `model`.
//!
//! Today: fixed-step time integration. Next: damped Newton on residual systems (trim
//! points, algebraic loops) using finite-difference Jacobians.

pub const integrate = @import("integrate.zig");

test {
    _ = @import("integrate.zig");
}
