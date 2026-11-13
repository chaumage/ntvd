//! Layer 2: architecture description and compilation. Depends on `base` and `components`.
//!
//!   Spec        plain data an engineer or a file produces
//!   compileInto validates it, or explains exactly what is wrong, and carves a plan
//!   Plan        immutable, resolved, evaluated as flat loops; `Workspace` holds one evaluation
//!
//! Storage is caller-provided and exactly sized: `bytesNeeded` gives the byte count, then
//! `compileInto` and `Workspace.into` carve their headers and arrays out of that buffer.

pub const spec = @import("spec.zig");
pub const storage = @import("storage.zig");
pub const plan = @import("plan.zig");
pub const compile_mod = @import("compile.zig");
/// Reference architectures, public so that upper layers can test against them.
pub const scenarios = @import("scenarios.zig");

pub const Spec = spec.Spec;
pub const ComponentSpec = spec.ComponentSpec;
pub const ConnectionSpec = spec.ConnectionSpec;
pub const ExternalSpec = spec.ExternalSpec;
pub const Component = spec.Component;

pub const Plan = plan.Plan;
pub const Workspace = plan.Workspace;

/// Alignment every plan and workspace buffer must satisfy.
pub const storage_alignment = storage.alignment;

pub const compileInto = compile_mod.compileInto;
pub const bytesNeeded = compile_mod.bytesNeeded;
pub const CompileError = compile_mod.CompileError;
pub const Diagnostic = @import("diagnostic.zig").Diagnostic;

test {
    _ = @import("path.zig");
    _ = @import("diagnostic.zig");
    _ = @import("storage.zig");
    _ = @import("plan.zig");
    _ = @import("compile.zig");
    _ = @import("scenarios.zig");
}
