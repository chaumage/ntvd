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

const contract = components.contract;

pub const Component = components.Component;

/// Where a component sits in the vehicle, and how it is turned. Both are relative to the body
/// frame: the vehicle's pose is the body's state, and the assembly is fixed.
pub const Placement = struct {
    /// Position of the component's origin in the body frame, metres.
    position_m: base.Vec3 = .{},
    /// Orientation of the local frame relative to the body frame, radians, intrinsic Z-Y-X
    /// (roll, pitch, yaw).
    attitude_rad: base.Vec3 = .{},
};

pub const ComponentSpec = struct {
    name: []const u8,
    component: Component,
    placement: Placement = .{},
    /// Initial values of the component's states, in declaration order. Empty means defaults:
    /// zero for scalars and vectors, identity for quaternions.
    initial_state: []const f64 = &.{},
};

/// A boundary signal: an operating condition or a trim variable chosen by the analysis.
pub const ExternalSpec = struct {
    name: []const u8,
    dim: base.Dim,
    /// Layout of the signal. A vector external declares its shape, quantity and frame; a
    /// scalar leaves the defaults.
    shape: contract.Shape = .scalar,
    quantity: contract.Quantity = .scalar,
    frame: contract.Frame = .none,
};

/// How a connection attaches its source to its target.
pub const Attachment = enum {
    /// A plain signal link. Cross-frame vectors are rotated by a generated node.
    signal,
    /// A rigid attachment of the source component to the vehicle: a wrench delivered to the
    /// body's mount point and translated by the source's placement.
    rigid,
};

pub const ConnectionSpec = struct {
    /// An external name, or "component.output".
    from: []const u8,
    /// "component.input".
    to: []const u8,
    /// Dimensionless factor applied to the signal, typically -1 to turn a drag magnitude into
    /// a force along the axis. Applied to every source of a summing input individually.
    gain: f64 = 1.0,
    attachment: Attachment = .signal,
};

pub const Spec = struct {
    components: []const ComponentSpec,
    externals: []const ExternalSpec = &.{},
    connections: []const ConnectionSpec = &.{},
};
