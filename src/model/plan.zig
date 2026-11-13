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

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const components = @import("components");
const path = @import("path.zig");

const limits = base.limits;
const Name = base.Name;
const Dim = base.Dim;
const Validity = base.Validity;
const Component = components.Component;

pub const Link = struct {
    /// Index into the signal array.
    source: u16,
    /// Factor applied to the source, typically 1 or -1.
    gain: f64,
};

/// The links driving one input port: `links[first .. first + count]`.
pub const InputWiring = struct {
    first: u16 = 0,
    count: u16 = 0,
};

pub const Node = struct {
    name: Name,
    component: Component,
    n_in: u8,
    n_out: u8,
    n_state: u8,
    /// Offset of this node's first state in the state array.
    state_offset: u16,
    /// Offset of this node's first output among the component outputs; the signal index is
    /// `n_externals + out_offset`.
    out_offset: u16,
    inputs: [limits.max_ports]InputWiring,
};

/// Scratch memory of one evaluation. Reusable across evaluations; owns the results.
pub const Workspace = struct {
    signals: [limits.max_signals]f64 = @splat(0.0),
    /// Validity of each node in the last `evaluate`, indexed by node.
    validity: [limits.max_components]Validity = @splat(.valid),
};

/// Only the first `n_*` entries of each array are meaningful.
pub const Plan = struct {
    n_externals: u16,
    external_names: [limits.max_externals]Name,
    external_dims: [limits.max_externals]Dim,

    n_nodes: u16,
    nodes: [limits.max_components]Node,
    /// Node indices in evaluation order.
    order: [limits.max_components]u16,

    n_links: u16,
    links: [limits.max_links]Link,

    n_states: u16,
    initial_state: [limits.max_states]f64,
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
        for (plan.nodes[0..plan.n_nodes]) |*node| {
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
        var u: [limits.max_ports]f64 = @splat(0.0);
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
        var u: [limits.max_ports]f64 = @splat(0.0);
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
        for (u, node.inputs[0..node.n_in]) |*value, wiring| {
            var acc: f64 = 0.0;
            for (plan.links[wiring.first..][0..wiring.count]) |link| {
                acc += link.gain * ws.signals[link.source];
            }
            value.* = acc;
        }
    }

    // -----------------------------------------------------------------------------------
    // Lookup by name, for analyses and reports. Not meant for inner loops.
    // -----------------------------------------------------------------------------------

    pub fn nodeIndex(plan: *const Plan, name: []const u8) ?u16 {
        for (plan.nodes[0..plan.n_nodes], 0..) |*node, i| {
            if (node.name.eql(name)) return @intCast(i);
        }
        return null;
    }

    pub fn externalIndex(plan: *const Plan, name: []const u8) ?u16 {
        for (plan.external_names[0..plan.n_externals], 0..) |*n, i| {
            if (n.eql(name)) return @intCast(i);
        }
        return null;
    }

    /// Signal index of "external" or "component.output".
    pub fn signalIndex(plan: *const Plan, text: []const u8) ?u16 {
        const p = path.parse(text) catch return null;
        const port = p.port orelse return plan.externalIndex(p.head);
        const ni = plan.nodeIndex(p.head) orelse return null;
        const node = &plan.nodes[ni];
        const oi = node.component.signature().outputIndex(port) orelse return null;
        return plan.n_externals + node.out_offset + @as(u16, @intCast(oi));
    }

    /// Index in the state array of "component.state".
    pub fn stateIndex(plan: *const Plan, text: []const u8) ?u16 {
        const p = path.parse(text) catch return null;
        const port = p.port orelse return null;
        const ni = plan.nodeIndex(p.head) orelse return null;
        const node = &plan.nodes[ni];
        const si = node.component.signature().stateIndex(port) orelse return null;
        return node.state_offset + @as(u16, @intCast(si));
    }
};

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
