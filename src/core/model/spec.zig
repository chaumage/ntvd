//! Declarative description of a vehicle architecture: plain data, no behavior.
//!
//! This is what an engineer, a file or a graphical editor produces. It is deliberately
//! unvalidated: `compile` turns it into a checked, immutable `Plan` or explains precisely
//! what is wrong.
//!
//! Wiring is written with paths:
//!   "wing.drag"  output port `drag` of the component named `wing`
//!   "wing.alpha" input port `alpha` of the component named `wing`
//!   "altitude"   an external signal (no dot), driven from outside the model
//!
//! Swapping a model for a better one means replacing the `component` value of one entry;
//! the connections do not change.

const base = @import("base");
const components = @import("components");

pub const Component = components.Component;

pub const ComponentSpec = struct {
    name: []const u8,
    component: Component,
    /// Initial values of the component's states, in declaration order. Empty means all zero.
    initial_state: []const f64 = &.{},
};

/// A boundary signal: an operating condition or a trim variable chosen by the analysis.
pub const ExternalSpec = struct {
    name: []const u8,
    dim: base.Dim,
};

pub const ConnectionSpec = struct {
    /// An external name, or "component.output".
    from: []const u8,
    /// "component.input".
    to: []const u8,
    /// Dimensionless factor applied to the signal, typically -1 to turn a drag magnitude into
    /// a force along the axis. Applied to every source of a summing input individually.
    gain: f64 = 1.0,
};

pub const Spec = struct {
    components: []const ComponentSpec,
    externals: []const ExternalSpec = &.{},
    connections: []const ConnectionSpec = &.{},
};
