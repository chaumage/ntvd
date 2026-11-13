//! The compiled model: immutable, fully resolved, evaluated as flat loops over indices.
//!
//! Produced by `compile`; never built by hand. All structural problems were rejected at
//! compile time, so evaluation has no error paths:
//!
//!   * every input port is wired as its signature demands,
//!   * every connection joins ports of identical dimension,
//!   * the nodes are in an order where every feedthrough component comes after its sources.
//!
//! Evaluation is *total*: for any input, even NaN or infinity, it returns finite signals and
//! a `Validity`. A component never sees a non-finite value; if one would reach it, the
//! component is skipped, its outputs are zero and the result is flagged `invalid`. This is
//! what lets solvers and optimizers probe nonsense without crashing the process.
//!
//! Signals and states
//!   signals   one flat array: the externals first, then the outputs of each component
//!             in declaration order
//!   states    one flat array: the states of each component in declaration order
//!
//! Dynamics are split in two phases so that an integrator or a nonlinear solver can use
//! either: `evaluate` (outputs, may include algebraic chains) then `derivatives`.
//! The time derivative of the whole model is `rhs`.
//!
//! Storage is caller-provided and exactly sized: `Plan` is a header of slices into the buffer
//! passed to `compileInto`, and `Workspace` is built the same way. See `storage.zig`.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const components = @import("components");
const path = @import("path.zig");
const storage_mod = @import("storage.zig");

const limits = base.limits;
const Name = base.Name;
const Dim = base.Dim;
const Validity = base.Validity;
const Component = components.Component;
const contract = components.contract;
const Sizer = storage_mod.Sizer;
const Carver = storage_mod.Carver;

pub const Link = struct {
    /// Index of the first slot in the signal array.
    source: u16,
    /// Factor applied to every slot of the source, typically 1 or -1.
    gain: f64,
};

/// The links driving one input port: `links[first .. first + count]`.
/// A wide port occupies `width` consecutive slots starting at `slot` in the node's input
/// buffer; every link carries that same width.
pub const InputWiring = struct {
    first: u16 = 0,
    count: u16 = 0,
    /// Offset of the port's first slot in the component input buffer.
    slot: u16 = 0,
    /// Number of consecutive slots the port occupies.
    width: u8 = 1,
};

pub const Node = struct {
    name: Name,
    component: Component,
    /// Total input slots of this node, the sum of the port widths.
    n_in: u8,
    /// Total output slots, the sum of the port widths.
    n_out: u8,
    /// Total state slots, the sum of the port widths.
    n_state: u8,
    /// Offset of this node's first state slot in the state array.
    state_offset: u16,
    /// Offset of this node's first output slot; the signal index is
    /// `n_externals + out_offset`.
    out_offset: u16,
    inputs: [limits.max_ports]InputWiring,
};

/// Only the first `n_*` entries of each array are meaningful. The arrays themselves are carved
/// from the caller's storage by `compileInto`.
pub const Plan = struct {
    external_names: []Name,
    external_dims: []Dim,

    nodes: []Node,
    /// Node indices in evaluation order.
    order: []u16,
    links: []Link,

    /// Initial state of the whole model, `n_states` long. Never written after compilation.
    initial_state: []const f64,
    /// State slots holding quaternions, normalized after each accepted step.
    attitude_states: []u16,

    n_externals: u16,
    n_nodes: u16,
    n_links: u16,
    n_states: u16,
    n_signals: u16,

    /// Computes every signal. Returns the worst validity among the nodes; per node details
    /// are in `ws.validity`.
    pub fn evaluate(
        plan: *const Plan,
        x: []const f64,
        externals: []const f64,
        ws: *Workspace,
    ) Validity {
        assert(x.len == plan.n_states);
        assert(externals.len == plan.n_externals);
        assert(ws.signals.len == plan.n_signals);
        assert(ws.validity.len == plan.n_nodes);
        @memcpy(ws.signals[0..plan.n_externals], externals);

        var worst: Validity = .valid;
        for (plan.order[0..plan.n_nodes]) |index| {
            const v = plan.evaluateNode(&plan.nodes[index], x, ws);
            ws.validity[index] = v;
            worst = worst.worst(v);
        }
        return worst;
    }

    /// Time derivative of every state. Requires `evaluate` to have filled `ws` for the same
    /// `x` and externals.
    pub fn derivatives(
        plan: *const Plan,
        x: []const f64,
        ws: *const Workspace,
        dx: []f64,
    ) Validity {
        assert(x.len == plan.n_states);
        assert(dx.len == plan.n_states);

        var worst: Validity = .valid;
        for (plan.nodes) |*node| {
            if (node.n_state == 0) continue;
            worst = worst.worst(plan.derivativesNode(node, x, ws, dx));
        }
        return worst;
    }

    /// `evaluate` followed by `derivatives`: dx/dt = f(x, externals).
    pub fn rhs(
        plan: *const Plan,
        x: []const f64,
        externals: []const f64,
        ws: *Workspace,
        dx: []f64,
    ) Validity {
        const outputs = plan.evaluate(x, externals, ws);
        return outputs.worst(plan.derivatives(x, ws, dx));
    }

    fn evaluateNode(plan: *const Plan, node: *const Node, x: []const f64, ws: *Workspace) Validity {
        const n_in: usize = node.n_in;
        var u: [limits.max_slots]f64 = @splat(0.0);
        // A component without feedthrough is evaluated before its sources may be computed,
        // and by contract ignores its inputs: it receives zeros.
        if (node.component.signature().feedthrough) plan.gather(node, ws, u[0..n_in]);

        const xs = x[node.state_offset..][0..node.n_state];
        const out = ws.signals[plan.n_externals + node.out_offset ..][0..node.n_out];
        if (!allFinite(xs) or !allFinite(u[0..n_in])) {
            @memset(out, 0.0);
            return .invalid;
        }
        const v = node.component.outputs(xs, u[0..n_in], out);
        if (!allFinite(out)) {
            @memset(out, 0.0);
            return .invalid;
        }
        return v;
    }

    fn derivativesNode(
        plan: *const Plan,
        node: *const Node,
        x: []const f64,
        ws: *const Workspace,
        dx: []f64,
    ) Validity {
        const n_in: usize = node.n_in;
        var u: [limits.max_slots]f64 = @splat(0.0);
        plan.gather(node, ws, u[0..n_in]);

        const xs = x[node.state_offset..][0..node.n_state];
        const d = dx[node.state_offset..][0..node.n_state];
        if (!allFinite(xs) or !allFinite(u[0..n_in])) {
            @memset(d, 0.0);
            return .invalid;
        }
        const v = node.component.derivatives(xs, u[0..n_in], d);
        if (!allFinite(d)) {
            @memset(d, 0.0);
            return .invalid;
        }
        return v;
    }

    fn gather(plan: *const Plan, node: *const Node, ws: *const Workspace, u: []f64) void {
        assert(u.len == node.n_in);
        @memset(u, 0.0);
        const sig = node.component.signature();
        for (0..sig.inputs.len) |pi| {
            const wiring = node.inputs[pi];
            const width: usize = wiring.width;
            const dest: usize = wiring.slot;
            assert(dest + width <= u.len);
            for (plan.links[wiring.first..][0..wiring.count]) |link| {
                const source: usize = link.source;
                for (0..width) |k| {
                    u[dest + k] += link.gain * ws.signals[source + k];
                }
            }
        }
    }

    // -----------------------------------------------------------------------------------
    // Lookup by name, for analyses and reports. Not meant for inner loops.
    // -----------------------------------------------------------------------------------

    pub fn nodeIndex(plan: *const Plan, name: []const u8) ?u16 {
        for (plan.nodes, 0..) |*node, i| {
            if (node.name.eql(name)) return @intCast(i);
        }
        return null;
    }

    pub fn externalIndex(plan: *const Plan, name: []const u8) ?u16 {
        for (plan.external_names, 0..) |*n, i| {
            if (n.eql(name)) return @intCast(i);
        }
        return null;
    }

    /// Signal index of "external" or "component.output". For a wide output this is its first
    /// slot; use `signalSlots` when the width matters.
    pub fn signalIndex(plan: *const Plan, text: []const u8) ?u16 {
        const p = path.parse(text) catch return null;
        const port = p.port orelse return plan.externalIndex(p.head);
        const ni = plan.nodeIndex(p.head) orelse return null;
        const node = &plan.nodes[ni];
        const sig = node.component.signature();
        const oi = sig.outputIndex(port) orelse return null;
        const slot = contract.slotsBefore(sig.outputs, oi);
        return plan.n_externals + node.out_offset + slot;
    }

    /// First signal index and slot width of "external" or "component.output".
    pub const SignalSlots = struct { index: u16, width: u8 };

    pub fn signalSlots(plan: *const Plan, text: []const u8) ?SignalSlots {
        const p = path.parse(text) catch return null;
        const port = p.port orelse {
            const i = plan.externalIndex(p.head) orelse return null;
            return .{ .index = i, .width = 1 };
        };
        const ni = plan.nodeIndex(p.head) orelse return null;
        const node = &plan.nodes[ni];
        const sig = node.component.signature();
        const oi = sig.outputIndex(port) orelse return null;
        const slot = contract.slotsBefore(sig.outputs, oi);
        return .{
            .index = plan.n_externals + node.out_offset + slot,
            .width = sig.outputs[oi].ptype.width(),
        };
    }

    /// Index of "component.state" in the state array. For a wide state this is its first slot.
    pub fn stateIndex(plan: *const Plan, text: []const u8) ?u16 {
        const p = path.parse(text) catch return null;
        const port = p.port orelse return null;
        const ni = plan.nodeIndex(p.head) orelse return null;
        const node = &plan.nodes[ni];
        const sig = node.component.signature();
        const si = sig.stateIndex(port) orelse return null;
        return node.state_offset + contract.slotsBefore(sig.states, si);
    }
};

/// Scratch memory of one evaluation. Reusable across evaluations; owns the results. The
/// arrays are carved from caller-provided storage by `into`.
pub const Workspace = struct {
    signals: []f64,
    /// Validity of each node in the last `evaluate`, indexed by node.
    validity: []Validity,

    /// Bytes needed for a workspace, including its header.
    pub fn bytesNeeded(plan: *const Plan) usize {
        var sizer: Sizer = .{};
        sizer.add(Workspace, 1);
        addTo(plan, &sizer);
        return sizer.bytes();
    }

    /// Contributes the sizes of the arrays, for a workspace embedded in a larger struct.
    pub fn addTo(plan: *const Plan, sizer: *Sizer) void {
        sizer.add(f64, plan.n_signals);
        sizer.add(Validity, plan.n_nodes);
    }

    /// Carves the arrays, initialized, and returns them as a value, for embedding.
    pub fn carve(plan: *const Plan, carver: *Carver) Workspace {
        const ws: Workspace = .{
            .signals = carver.take(f64, plan.n_signals),
            .validity = carver.take(Validity, plan.n_nodes),
        };
        @memset(ws.signals, 0.0);
        @memset(ws.validity, .valid);
        return ws;
    }

    /// Builds a workspace in `storage`, which must be at least `bytesNeeded(plan)` long.
    pub fn into(plan: *const Plan, storage: storage_mod.Buffer) *Workspace {
        var carver = Carver.init(storage);
        const ws = carver.takeOne(Workspace);
        ws.* = carve(plan, &carver);
        assert(carver.cursor == bytesNeeded(plan));
        return ws;
    }
};

pub const storage_alignment = storage_mod.alignment;

comptime {
    if (@alignOf(Plan) > storage_alignment) @compileError("Plan alignment exceeds storage");
    if (@alignOf(Workspace) > storage_alignment) {
        @compileError("Workspace alignment exceeds storage");
    }
    if (@alignOf(Node) > storage_alignment) @compileError("Node alignment exceeds storage");
    if (@alignOf(Link) > storage_alignment) @compileError("Link alignment exceeds storage");
    if (@alignOf(Validity) > storage_alignment) @compileError("Validity alignment exceeds storage");
}

fn allFinite(values: []const f64) bool {
    for (values) |v| {
        if (!std.math.isFinite(v)) return false;
    }
    return true;
}

test "allFinite" {
    try std.testing.expect(allFinite(&.{}));
    try std.testing.expect(allFinite(&.{ 1.0, -2.0 }));
    try std.testing.expect(!allFinite(&.{ 1.0, std.math.nan(f64) }));
    try std.testing.expect(!allFinite(&.{std.math.inf(f64)}));
}
