//! Layer 4: steady analyses built on the compiled model and the numerical solvers.
//!
//! `trim` finds steady-flight points; envelope sweeps will build on it. Like every layer,
//! this one only imports the layers below it (`base`, `model`, `solve`).

pub const trim = @import("trim.zig");

test {
    _ = @import("trim.zig");
}
