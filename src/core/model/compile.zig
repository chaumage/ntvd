//! Spec -> Plan. Rejects every structural mistake with a precise error and a diagnostic.
//!
//! Stages (each a small function, run in this order):
//!   1. measure       exact layout and byte count of the plan
//!   2. externals     names and typed boundary signals
//!   3. nodes         authored components, placements and initial states
//!   4. connections   paths resolve, types agree; rigid attachments and cross-frame vectors
//!                    become generated nodes
//!   5. drivers       every `single` input has exactly one source
//!   6. links         connections laid out per input port
//!   7. order         feedthrough components sorted after their sources
//!
//! The plan is carved from a caller-owned buffer. `bytesNeeded` and `compileInto` share the
//! same spec-only connection classifier, so the required size and the actual layout cannot
//! drift. A connection between two different frames becomes an ordinary `frame_transform`
//! node, which inherits evaluation, validity and ordering from the rest of the graph.
//!
//! Algebraic loops (feedthrough components feeding each other in a cycle) are rejected for
//! now. The way to break one is to make a state of it (a dynamic component) or to turn the
//! loop variable into an external that an analysis solves for.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const components = @import("components");
const contract = components.contract;
const frame_transform = components.frame_transform;
const spec_mod = @import("spec.zig");
const plan_mod = @import("plan.zig");
const storage_mod = @import("storage.zig");
const path = @import("path.zig");
const Diagnostic = @import("diagnostic.zig").Diagnostic;

const limits = base.limits;
const Spec = spec_mod.Spec;
const Plan = plan_mod.Plan;
const Node = plan_mod.Node;
const Link = plan_mod.Link;
const Name = base.Name;
const Sizer = storage_mod.Sizer;
const Carver = storage_mod.Carver;

pub const CompileError = error{
    BufferTooSmall,
    TooManyComponents,
    TooManyExternals,
    TooManyConnections,
    TooManyStates,
    TooManySignals,
    TooManyGeneratedNodes,
    InvalidName,
    DuplicateName,
    InvalidParameter,
    BadInitialState,
    BadPlacement,
    BadPath,
    UnknownComponent,
    UnknownExternal,
    UnknownPort,
    FrameMismatch,
    DimensionMismatch,
    InvalidGain,
    MultiplyDriven,
    UnconnectedInput,
    AlgebraicLoop,
    AmbiguousBody,
    BodyHasNoAttitude,
    RigidAttachmentNotWrench,
    RigidAttachmentToNonBody,
};

/// A connection with all names resolved and its frame transform decided, if any.
const Resolved = struct {
    to_node: u16,
    to_port: u8,
    source: u16,
    /// Producing node, null when the source is an external.
    from_node: ?u16,
    gain: f64,
};

comptime {
    if (@alignOf(Resolved) > storage_mod.alignment) {
        @compileError("Resolved alignment exceeds storage");
    }
    if (@alignOf(Name) > storage_mod.alignment) {
        @compileError("Name alignment exceeds storage");
    }
    if (@alignOf(contract.PortType) > storage_mod.alignment) {
        @compileError("PortType alignment exceeds storage");
    }
}

/// Counts and byte size of one plan. `usize` so the size can be computed before any ceiling
/// check; `compileInto` rejects anything that does not fit the `u16` encoding.
pub const Layout = struct {
    n_externals: usize,
    n_external_slots: usize,
    n_nodes: usize,
    n_generated: usize,
    n_links: usize,
    n_states: usize,
    n_signals: usize,
    n_attitude_states: usize,
    bytes: usize,
};

/// Exact byte count for `spec` if it compiles; a safe upper bound otherwise.
pub fn bytesNeeded(spec: *const Spec) usize {
    return measure(spec).bytes;
}

/// Builds a plan in `storage` and returns a pointer to the header at its start. The storage
/// must outlive the plan and must not be reused while the plan is alive.
pub fn compileInto(
    storage: storage_mod.Buffer,
    spec: *const Spec,
    diag: *Diagnostic,
) CompileError!*Plan {
    diag.* = .{};
    const layout = measure(spec);
    if (storage.len < layout.bytes) {
        diag.set("plan storage needs {d} bytes, got {d}", .{ layout.bytes, storage.len });
        return error.BufferTooSmall;
    }
    try checkCounts(layout, diag);

    var carver = Carver.init(storage);
    const plan = carver.takeOne(Plan);
    const external_names = carver.take(Name, layout.n_externals);
    const external_ptypes = carver.take(contract.PortType, layout.n_externals);
    const external_offsets = carver.take(u16, layout.n_externals);
    const nodes = carver.take(Node, layout.n_nodes);
    const order = carver.take(u16, layout.n_nodes);
    const links = carver.take(Link, layout.n_links);
    const initial_state = carver.take(f64, layout.n_states);
    const attitude_states = carver.take(u16, layout.n_attitude_states);
    const resolved = carver.take(Resolved, layout.n_links);
    const waiting = carver.take(u16, layout.n_nodes);
    const placed = carver.take(bool, layout.n_nodes);
    assert(carver.cursor == layout.bytes);

    plan.* = .{
        .external_names = external_names,
        .external_ptypes = external_ptypes,
        .external_offsets = external_offsets,
        .nodes = nodes,
        .order = order,
        .links = links,
        .initial_state = initial_state,
        .attitude_states = attitude_states,
        .n_externals = @intCast(layout.n_externals),
        .n_external_slots = @intCast(layout.n_external_slots),
        .n_nodes = @intCast(layout.n_nodes),
        .n_links = @intCast(layout.n_links),
        .n_states = @intCast(layout.n_states),
        .n_signals = @intCast(layout.n_signals),
    };

    try buildExternals(spec, plan, diag);
    try buildNodes(spec, plan, initial_state, attitude_states, diag);
    const body = try findBody(spec, diag);
    try checkRigidAttachments(spec, diag);
    const n = try buildResolved(spec, plan, body, resolved, diag);
    assert(n == plan.n_links);
    try checkDrivers(plan, resolved[0..n], diag);
    layoutLinks(plan, resolved[0..n]);
    try orderNodes(plan, resolved[0..n], waiting, placed, diag);
    return plan;
}

const Count = struct { nodes: usize, links: usize };

/// Counts and byte size only. Never fails: an invalid spec still has a computable footprint.
/// A connection that cannot be classified is charged the largest transform footprint, so the
/// estimate is never below what `compileInto` would need.
fn measure(spec: *const Spec) Layout {
    var n_states: usize = 0;
    var n_outputs: usize = 0;
    var n_attitude_states: usize = 0;
    for (spec.components) |cs| {
        const sig = cs.component.signature();
        n_states += @as(usize, sig.stateSlots());
        n_outputs += @as(usize, sig.outputSlots());
        for (sig.states) |p| {
            if (p.ptype.quantity == .quaternion) n_attitude_states += 1;
        }
    }
    var n_external_slots: usize = 0;
    for (spec.externals) |ext| n_external_slots += @as(usize, externalType(ext).width());

    const body = findBodyLenient(spec);
    var n_generated: usize = 0;
    var n_links: usize = 0;
    for (spec.connections) |c| {
        const count = countOne(spec, c, body) orelse Count{ .nodes = 1, .links = 3 };
        n_generated += count.nodes;
        n_links += count.links;
    }
    const ft = frame_transform.Model.signature;
    const generated_outputs = n_generated * @as(usize, ft.outputSlots());
    const n_nodes = spec.components.len + n_generated;
    const n_signals = n_external_slots + n_outputs + generated_outputs;

    var sizer: Sizer = .{};
    sizer.add(Plan, 1);
    sizer.add(Name, spec.externals.len);
    sizer.add(contract.PortType, spec.externals.len);
    sizer.add(u16, spec.externals.len);
    sizer.add(Node, n_nodes);
    sizer.add(u16, n_nodes);
    sizer.add(Link, n_links);
    sizer.add(f64, n_states);
    sizer.add(u16, n_attitude_states);
    sizer.add(Resolved, n_links);
    sizer.add(u16, n_nodes);
    sizer.add(bool, n_nodes);

    return .{
        .n_externals = spec.externals.len,
        .n_external_slots = n_external_slots,
        .n_nodes = n_nodes,
        .n_generated = n_generated,
        .n_links = n_links,
        .n_states = n_states,
        .n_signals = n_signals,
        .n_attitude_states = n_attitude_states,
        .bytes = sizer.bytes(),
    };
}

fn countOne(spec: *const Spec, c: spec_mod.ConnectionSpec, body: ?Body) ?Count {
    var diag: Diagnostic = .{};
    const cl = classifyOne(spec, c, body, &diag) catch return null;
    if (cl.transform) |t| {
        return .{ .nodes = 1, .links = if (t.needs_attitude) 3 else 2 };
    }
    return .{ .nodes = 0, .links = 1 };
}

fn checkCounts(layout: Layout, diag: *Diagnostic) CompileError!void {
    const ceiling: usize = limits.max_index;
    if (layout.n_externals > ceiling) {
        diag.set("{d} externals exceed the u16 encoding limit of {d}", .{
            layout.n_externals, ceiling,
        });
        return error.TooManyExternals;
    }
    if (layout.n_external_slots > ceiling) {
        diag.set("{d} external slots exceed the u16 encoding limit of {d}", .{
            layout.n_external_slots, ceiling,
        });
        return error.TooManyExternals;
    }
    if (layout.n_nodes > ceiling) {
        diag.set("{d} components exceed the u16 encoding limit of {d}", .{
            layout.n_nodes, ceiling,
        });
        return error.TooManyComponents;
    }
    if (layout.n_generated > ceiling) {
        diag.set("{d} generated nodes exceed the u16 encoding limit of {d}", .{
            layout.n_generated, ceiling,
        });
        return error.TooManyGeneratedNodes;
    }
    if (layout.n_links > ceiling) {
        diag.set("{d} links exceed the u16 encoding limit of {d}", .{
            layout.n_links, ceiling,
        });
        return error.TooManyConnections;
    }
    if (layout.n_states > ceiling) {
        diag.set("{d} states exceed the u16 encoding limit of {d}", .{
            layout.n_states, ceiling,
        });
        return error.TooManyStates;
    }
    if (layout.n_signals > ceiling) {
        diag.set("{d} signals exceed the u16 encoding limit of {d}", .{
            layout.n_signals, ceiling,
        });
        return error.TooManySignals;
    }
}

fn makeName(text: []const u8, what: []const u8, diag: *Diagnostic) CompileError!base.Name {
    if (std.mem.startsWith(u8, text, "__x_")) {
        diag.set("invalid {s} name '{s}': the '__x_' prefix is reserved", .{ what, text });
        return error.InvalidName;
    }
    return base.Name.init(text) catch |err| {
        diag.set("invalid {s} name '{s}': {s}", .{ what, text, @errorName(err) });
        return error.InvalidName;
    };
}

fn externalType(ext: spec_mod.ExternalSpec) contract.PortType {
    return .{ .dim = ext.dim, .shape = ext.shape, .quantity = ext.quantity, .frame = ext.frame };
}

fn buildExternals(spec: *const Spec, plan: *Plan, diag: *Diagnostic) CompileError!void {
    var slot: u16 = 0;
    for (spec.externals, 0..) |ext, i| {
        plan.external_names[i] = try makeName(ext.name, "external", diag);
        for (spec.externals[0..i]) |other| {
            if (std.mem.eql(u8, other.name, ext.name)) {
                diag.set("external '{s}' is declared twice", .{ext.name});
                return error.DuplicateName;
            }
        }
        const pt = externalType(ext);
        if (!contract.isValidPortType(pt)) {
            diag.set("external '{s}' has an invalid shape, quantity or frame", .{ext.name});
            return error.FrameMismatch;
        }
        plan.external_ptypes[i] = pt;
        plan.external_offsets[i] = slot;
        slot += pt.width();
    }
    assert(slot == plan.n_external_slots);
}

fn buildNodes(
    spec: *const Spec,
    plan: *Plan,
    initial_state: []f64,
    attitude_states: []u16,
    diag: *Diagnostic,
) CompileError!void {
    var state_cursor: u16 = 0;
    var out_cursor: u16 = 0;
    for (spec.components, 0..) |cs, i| {
        const name = try makeName(cs.name, "component", diag);
        for (spec.components[0..i]) |other| {
            if (std.mem.eql(u8, other.name, cs.name)) {
                diag.set("component '{s}' is declared twice", .{cs.name});
                return error.DuplicateName;
            }
        }
        if (cs.component.validate()) |problem| {
            diag.set("component '{s}' ({s}/{s}): {s}", .{
                cs.name, cs.component.kindName(), cs.component.modelName(), problem,
            });
            return error.InvalidParameter;
        }
        const placement = resolvePlacement(cs.placement) orelse {
            diag.set("component '{s}' has a non-finite placement", .{cs.name});
            return error.BadPlacement;
        };

        const sig = cs.component.signature();
        const node = &plan.nodes[i];
        const in_slots = sig.inputSlots();
        const out_slots = sig.outputSlots();
        const state_slots = sig.stateSlots();
        node.name = name;
        node.component = cs.component;
        node.placement = placement;
        node.n_in = @intCast(in_slots);
        node.n_out = @intCast(out_slots);
        node.n_state = @intCast(state_slots);
        node.state_offset = state_cursor;
        node.out_offset = out_cursor;
        node.inputs = @splat(.{});

        try copyInitialState(cs, sig, initial_state, state_cursor, diag);
        state_cursor += state_slots;
        out_cursor += out_slots;
    }
    assert(state_cursor == plan.n_states);
    fillAttitudeStates(plan, spec.components.len, attitude_states);
}

fn fillAttitudeStates(plan: *const Plan, n_authored: usize, out: []u16) void {
    var cursor: usize = 0;
    for (plan.nodes[0..n_authored]) |*node| {
        const sig = node.component.signature();
        for (sig.states, 0..) |p, si| {
            if (p.ptype.quantity != .quaternion) continue;
            out[cursor] = node.state_offset + contract.slotsBefore(sig.states, @intCast(si));
            cursor += 1;
        }
    }
    assert(cursor == out.len);
}

fn copyInitialState(
    cs: spec_mod.ComponentSpec,
    sig: *const contract.Signature,
    state: []f64,
    offset: u16,
    diag: *Diagnostic,
) CompileError!void {
    const n: usize = sig.stateSlots();
    const dest = state[offset..][0..n];
    if (cs.initial_state.len == 0) {
        fillStateDefaults(sig, dest);
        return;
    }
    if (cs.initial_state.len != n) {
        diag.set("component '{s}' has {d} states but {d} initial values", .{
            cs.name, n, cs.initial_state.len,
        });
        return error.BadInitialState;
    }
    for (cs.initial_state) |v| {
        if (!std.math.isFinite(v)) {
            diag.set("component '{s}' has a non-finite initial state", .{cs.name});
            return error.BadInitialState;
        }
    }
    @memcpy(dest, cs.initial_state);
    try normalizeQuaternions(cs.name, sig, dest, diag);
}

fn fillStateDefaults(sig: *const contract.Signature, dest: []f64) void {
    var cursor: usize = 0;
    for (sig.states) |p| {
        const width: usize = p.ptype.width();
        @memset(dest[cursor..][0..width], 0.0);
        // A quaternion defaults to the identity rotation, (w, x, y, z) = (1, 0, 0, 0).
        if (p.ptype.quantity == .quaternion) dest[cursor] = 1.0;
        cursor += width;
    }
}

fn normalizeQuaternions(
    name: []const u8,
    sig: *const contract.Signature,
    dest: []f64,
    diag: *Diagnostic,
) CompileError!void {
    var cursor: usize = 0;
    for (sig.states) |p| {
        const width: usize = p.ptype.width();
        if (p.ptype.quantity == .quaternion) {
            const q: base.Quat = .{
                .w = dest[cursor],
                .x = dest[cursor + 1],
                .y = dest[cursor + 2],
                .z = dest[cursor + 3],
            };
            const unit = base.Quat.normalized(q) orelse {
                diag.set("component '{s}' has a zero or non-finite initial '{s}'", .{
                    name, p.name,
                });
                return error.BadInitialState;
            };
            dest[cursor] = unit.w;
            dest[cursor + 1] = unit.x;
            dest[cursor + 2] = unit.y;
            dest[cursor + 3] = unit.z;
        }
        cursor += width;
    }
}

// ---------------------------------------------------------------------------------------
// Frame resolution: the same classifier backs `measure` and `compileInto`.
// ---------------------------------------------------------------------------------------

const FrameRef = union(enum) {
    none,
    world,
    body,
    local: u16,
};

const SpecEndpoint = struct {
    /// Owning component, null for an external signal.
    node: ?u16,
    /// Output port index, input port index, or external index when `node` is null.
    index: u16,
    ptype: contract.PortType,
    frame: FrameRef,
};

const Transform = struct {
    apply: frame_transform.Apply,
    /// True when the transform reads the body attitude through its `q` input.
    needs_attitude: bool,
};

const Classified = struct {
    from: SpecEndpoint,
    to: SpecEndpoint,
    gain: f64,
    transform: ?Transform,
};

const Body = struct {
    node: u16,
    /// Index of the body's wrench summing input `mount`.
    mount_port: u8,
    attitude_port: ?u32,
};

fn classifyOne(
    spec: *const Spec,
    c: spec_mod.ConnectionSpec,
    body: ?Body,
    diag: *Diagnostic,
) CompileError!Classified {
    if (!std.math.isFinite(c.gain)) {
        diag.set("connection {s} -> {s} has a non-finite gain", .{ c.from, c.to });
        return error.InvalidGain;
    }
    if (c.attachment == .rigid) return classifyRigid(spec, c, body, diag);
    const from = try specSource(spec, c.from, diag);
    const to = try specTarget(spec, c.to, diag);
    try checkTypes(c, from, to, diag);
    const transform = try frameTransform(spec, from, to, body, diag);
    return .{ .from = from, .to = to, .gain = c.gain, .transform = transform };
}

/// A rigid attachment is one wire: a component's local wrench output to the designated
/// body's `mount` input. It becomes a single `mount` node, placed at the source.
fn classifyRigid(
    spec: *const Spec,
    c: spec_mod.ConnectionSpec,
    body: ?Body,
    diag: *Diagnostic,
) CompileError!Classified {
    const from = try specSource(spec, c.from, diag);
    const to = try specTarget(spec, c.to, diag);
    const source_node = from.node orelse {
        diag.set("rigid attachment {s} -> {s} needs a component's wrench output", .{
            c.from, c.to,
        });
        return error.RigidAttachmentNotWrench;
    };
    if (from.ptype.shape != .wrench) {
        diag.set("rigid attachment {s} -> {s} needs a wrench output", .{ c.from, c.to });
        return error.RigidAttachmentNotWrench;
    }
    if (from.ptype.frame != .local) {
        diag.set("rigid attachment {s} -> {s} needs a local-frame wrench output", .{
            c.from, c.to,
        });
        return error.FrameMismatch;
    }
    const b = body orelse {
        diag.set("rigid attachment {s} -> {s} needs exactly one body", .{ c.from, c.to });
        return error.AmbiguousBody;
    };
    if (to.node != b.node or to.index != b.mount_port) {
        diag.set("rigid attachment {s} -> {s} must target the body's mount input", .{
            c.from, c.to,
        });
        return error.RigidAttachmentToNonBody;
    }
    const placement = try placementOf(spec, source_node, diag);
    return .{
        .from = from,
        .to = to,
        .gain = c.gain,
        .transform = .{
            .apply = .{
                .mode = .mount,
                .pre = placement.orientation,
                .dynamic = .none,
                .r = placement.position,
            },
            .needs_attitude = false,
        },
    };
}

fn checkTypes(
    c: spec_mod.ConnectionSpec,
    from: SpecEndpoint,
    to: SpecEndpoint,
    diag: *Diagnostic,
) CompileError!void {
    if (from.ptype.shape != to.ptype.shape) {
        diag.set("{s} cannot drive {s}: incompatible shape", .{ c.from, c.to });
        return error.FrameMismatch;
    }
    if (from.ptype.quantity != to.ptype.quantity) {
        diag.set("{s} cannot drive {s}: incompatible quantity", .{ c.from, c.to });
        return error.FrameMismatch;
    }
    if (from.ptype.shape != .wrench and !from.ptype.dim.eql(to.ptype.dim)) {
        diag.set("{s} [{f}] cannot drive {s} [{f}]", .{
            c.from, from.ptype.dim, c.to, to.ptype.dim,
        });
        return error.DimensionMismatch;
    }
}

fn frameTransform(
    spec: *const Spec,
    from: SpecEndpoint,
    to: SpecEndpoint,
    body: ?Body,
    diag: *Diagnostic,
) CompileError!?Transform {
    if (sameFrame(from.frame, to.frame)) return null;
    if (isNone(from.frame) or isNone(to.frame)) {
        diag.set("a scalar signal has no frame and cannot cross a frame boundary", .{});
        return error.FrameMismatch;
    }
    switch (from.ptype.shape) {
        .scalar => {
            diag.set("a scalar signal has no frame and cannot cross a frame boundary", .{});
            return error.FrameMismatch;
        },
        .quat => {
            diag.set("quaternion frame transforms need the body rate; not available yet", .{});
            return error.FrameMismatch;
        },
        .wrench => {
            diag.set("a wrench may only be driven by a rigid attachment", .{});
            return error.FrameMismatch;
        },
        .vec3 => {},
    }
    if (from.ptype.quantity == .position) {
        diag.set("position transforms need the body position state; not available yet", .{});
        return error.FrameMismatch;
    }

    var pre = base.Quat.identity;
    var post = base.Quat.identity;
    if (localNode(to.frame)) |n| {
        pre = base.Quat.conjugate(try orientationOf(spec, n, diag));
    }
    if (localNode(from.frame)) |n| post = try orientationOf(spec, n, diag);

    const dynamic: frame_transform.Dynamic =
        if (isNonWorld(from.frame) and isNonWorld(to.frame))
            .none
        else if (isNonWorld(from.frame))
            .direct
        else if (isNonWorld(to.frame))
            .conjugate
        else
            .none;
    if (dynamic != .none) {
        const b = body orelse {
            diag.set("a world-frame transform needs exactly one body component", .{});
            return error.AmbiguousBody;
        };
        if (b.attitude_port == null) {
            diag.set("the body has no 'attitude' output for world-frame transforms", .{});
            return error.BodyHasNoAttitude;
        }
    }
    return .{
        .apply = .{
            .mode = .rotate,
            .pre = pre,
            .post = post,
            .dynamic = dynamic,
        },
        .needs_attitude = dynamic != .none,
    };
}

fn sameFrame(a: FrameRef, b: FrameRef) bool {
    if (std.meta.activeTag(a) != std.meta.activeTag(b)) return false;
    return switch (a) {
        .none, .world, .body => true,
        .local => |na| na == b.local,
    };
}

fn isNone(f: FrameRef) bool {
    return std.meta.activeTag(f) == .none;
}

fn isNonWorld(f: FrameRef) bool {
    return switch (f) {
        .none, .world => false,
        .body, .local => true,
    };
}

fn localNode(f: FrameRef) ?u16 {
    return switch (f) {
        .local => |n| n,
        else => null,
    };
}

fn resolvePlacement(p: spec_mod.Placement) ?base.Placement {
    if (!p.position_m.isFinite() or !p.attitude_rad.isFinite()) return null;
    return .{
        .position = p.position_m,
        .orientation = base.Quat.fromEuler(p.attitude_rad.x, p.attitude_rad.y, p.attitude_rad.z),
    };
}

fn placementOf(spec: *const Spec, node: u16, diag: *Diagnostic) CompileError!base.Placement {
    return resolvePlacement(spec.components[node].placement) orelse {
        diag.set("component '{s}' has a non-finite placement", .{
            spec.components[node].name,
        });
        return error.BadPlacement;
    };
}

fn orientationOf(spec: *const Spec, node: u16, diag: *Diagnostic) CompileError!base.Quat {
    return (try placementOf(spec, node, diag)).orientation;
}

fn frameOf(
    pt: contract.PortType,
    node: ?u16,
    text: []const u8,
    diag: *Diagnostic,
) CompileError!FrameRef {
    return switch (pt.frame) {
        .none => .none,
        .world => .world,
        .body => .body,
        .local => if (node) |n| .{ .local = n } else {
            diag.set("'{s}' is in a local frame but has no owning component", .{text});
            return error.FrameMismatch;
        },
    };
}

fn specSource(spec: *const Spec, text: []const u8, diag: *Diagnostic) CompileError!SpecEndpoint {
    const p = path.parse(text) catch {
        diag.set("malformed source path '{s}'", .{text});
        return error.BadPath;
    };
    const port = p.port orelse {
        const ei = specExternalIndex(spec, p.head) orelse {
            diag.set("unknown external signal '{s}'", .{p.head});
            return error.UnknownExternal;
        };
        const pt = externalType(spec.externals[ei]);
        return .{
            .node = null,
            .index = ei,
            .ptype = pt,
            .frame = try frameOf(pt, null, text, diag),
        };
    };
    const ni = specNodeIndex(spec, p.head) orelse {
        diag.set("unknown component '{s}' in '{s}'", .{ p.head, text });
        return error.UnknownComponent;
    };
    const sig = spec.components[ni].component.signature();
    const oi = sig.outputIndex(port) orelse {
        diag.set("component '{s}' has no output '{s}'", .{ p.head, port });
        return error.UnknownPort;
    };
    const pt = sig.outputs[oi].ptype;
    return .{
        .node = ni,
        .index = @intCast(oi),
        .ptype = pt,
        .frame = try frameOf(pt, ni, text, diag),
    };
}

fn specTarget(spec: *const Spec, text: []const u8, diag: *Diagnostic) CompileError!SpecEndpoint {
    const p = path.parse(text) catch {
        diag.set("malformed target path '{s}'", .{text});
        return error.BadPath;
    };
    const port = p.port orelse {
        diag.set("target '{s}' must be written component.input", .{text});
        return error.BadPath;
    };
    const ni = specNodeIndex(spec, p.head) orelse {
        diag.set("unknown component '{s}' in '{s}'", .{ p.head, text });
        return error.UnknownComponent;
    };
    const sig = spec.components[ni].component.signature();
    const ii = sig.inputIndex(port) orelse {
        diag.set("component '{s}' has no input '{s}'", .{ p.head, port });
        return error.UnknownPort;
    };
    const pt = sig.inputs[ii].ptype;
    return .{
        .node = ni,
        .index = @intCast(ii),
        .ptype = pt,
        .frame = try frameOf(pt, ni, text, diag),
    };
}

fn specNodeIndex(spec: *const Spec, name: []const u8) ?u16 {
    for (spec.components, 0..) |cs, i| {
        if (std.mem.eql(u8, cs.name, name)) return @intCast(i);
    }
    return null;
}

fn specExternalIndex(spec: *const Spec, name: []const u8) ?u16 {
    for (spec.externals, 0..) |ext, i| {
        if (std.mem.eql(u8, ext.name, name)) return @intCast(i);
    }
    return null;
}

fn findBody(spec: *const Spec, diag: *Diagnostic) CompileError!?Body {
    var found: ?Body = null;
    for (spec.components, 0..) |cs, i| {
        const sig = cs.component.signature();
        const mi = sig.inputIndex("mount") orelse continue;
        if (sig.inputs[mi].ptype.shape != .wrench) continue;
        if (found != null) {
            diag.set("more than one component has a wrench mount input; body is ambiguous", .{});
            return error.AmbiguousBody;
        }
        found = .{
            .node = @intCast(i),
            .mount_port = @intCast(mi),
            .attitude_port = sig.outputIndex("attitude"),
        };
    }
    return found;
}

fn findBodyLenient(spec: *const Spec) ?Body {
    for (spec.components, 0..) |cs, i| {
        const sig = cs.component.signature();
        const mi = sig.inputIndex("mount") orelse continue;
        if (sig.inputs[mi].ptype.shape != .wrench) continue;
        return .{
            .node = @intCast(i),
            .mount_port = @intCast(mi),
            .attitude_port = sig.outputIndex("attitude"),
        };
    }
    return null;
}

/// A component may be rigidly attached once. `body.mount` sums, so a second wire would
/// otherwise be silently added a second time. Reject it like any other multiply-driven input.
fn checkRigidAttachments(spec: *const Spec, diag: *Diagnostic) CompileError!void {
    for (spec.connections, 0..) |c, i| {
        if (c.attachment != .rigid) continue;
        const node = rigidSourceNode(spec, c.from) orelse continue;
        for (spec.connections[0..i]) |earlier| {
            if (earlier.attachment != .rigid) continue;
            if (rigidSourceNode(spec, earlier.from) != node) continue;
            diag.set("component '{s}' is rigidly attached more than once", .{
                spec.components[node].name,
            });
            return error.MultiplyDriven;
        }
    }
}

fn rigidSourceNode(spec: *const Spec, text: []const u8) ?u16 {
    const p = path.parse(text) catch return null;
    if (p.port == null) return null;
    return specNodeIndex(spec, p.head);
}

// ---------------------------------------------------------------------------------------
// Resolved links, including the generated transform nodes.
// ---------------------------------------------------------------------------------------

const BuildState = struct {
    resolved: []Resolved,
    cursor: u16 = 0,
    node_cursor: u16,
    out_cursor: u16,
    generated: u16 = 0,
};

fn buildResolved(
    spec: *const Spec,
    plan: *Plan,
    body: ?Body,
    resolved: []Resolved,
    diag: *Diagnostic,
) CompileError!u32 {
    var st: BuildState = .{
        .resolved = resolved,
        .node_cursor = @intCast(spec.components.len),
        .out_cursor = authoredOutputSlots(plan, spec.components.len),
    };
    for (spec.connections) |c| {
        const cl = try classifyOne(spec, c, body, diag);
        if (cl.transform) |t| {
            try appendTransform(spec, plan, body, cl, t, &st, diag);
        } else {
            append(&st, .{
                .to_node = cl.to.node.?,
                .to_port = @intCast(cl.to.index),
                .source = sourceSlot(plan, cl.from),
                .from_node = cl.from.node,
                .gain = cl.gain,
            });
        }
    }
    assert(st.node_cursor == plan.n_nodes);
    assert(plan.n_external_slots + st.out_cursor == plan.n_signals);
    return st.cursor;
}

fn append(st: *BuildState, r: Resolved) void {
    st.resolved[st.cursor] = r;
    st.cursor += 1;
}

fn authoredOutputSlots(plan: *const Plan, n_authored: usize) u16 {
    var n: u16 = 0;
    for (plan.nodes[0..n_authored]) |node| n = node.out_offset + node.n_out;
    return n;
}

fn appendTransform(
    spec: *const Spec,
    plan: *Plan,
    body: ?Body,
    cl: Classified,
    t: Transform,
    st: *BuildState,
    diag: *Diagnostic,
) CompileError!void {
    const sig = frame_transform.Model.signature;
    const x_node = st.node_cursor;
    const x = &plan.nodes[x_node];
    x.* = .{
        .name = try generatedName(st.generated, diag),
        .component = .{ .frame_transform = .{ .apply = t.apply } },
        .placement = .{},
        .n_in = @intCast(sig.inputSlots()),
        .n_out = @intCast(sig.outputSlots()),
        .n_state = 0,
        .state_offset = 0,
        .out_offset = st.out_cursor,
        .inputs = @splat(.{}),
    };
    const port_name = if (t.apply.mode == .rotate) "v" else "wrench";
    append(st, .{
        .to_node = x_node,
        .to_port = @intCast(sig.inputIndex(port_name).?),
        .source = sourceSlot(plan, cl.from),
        .from_node = cl.from.node,
        .gain = cl.gain,
    });
    append(st, .{
        .to_node = cl.to.node.?,
        .to_port = @intCast(cl.to.index),
        .source = plan.n_external_slots + st.out_cursor +
            contract.slotsBefore(sig.outputs, sig.outputIndex(port_name).?),
        .from_node = x_node,
        .gain = 1.0,
    });
    if (t.needs_attitude) try appendAttitude(spec, plan, body.?, x_node, sig, st, diag);
    st.generated += 1;
    st.node_cursor += 1;
    st.out_cursor += sig.outputSlots();
}

fn appendAttitude(
    spec: *const Spec,
    plan: *Plan,
    b: Body,
    x_node: u16,
    sig: contract.Signature,
    st: *BuildState,
    diag: *Diagnostic,
) CompileError!void {
    _ = spec;
    const b_node = &plan.nodes[b.node];
    const b_sig = b_node.component.signature();
    const b_port = b.attitude_port orelse {
        diag.set("the body has no 'attitude' output for world-frame transforms", .{});
        return error.BodyHasNoAttitude;
    };
    append(st, .{
        .to_node = x_node,
        .to_port = @intCast(sig.inputIndex("q").?),
        .source = plan.n_external_slots + b_node.out_offset +
            contract.slotsBefore(b_sig.outputs, b_port),
        .from_node = b.node,
        .gain = 1.0,
    });
}

fn sourceSlot(plan: *const Plan, ep: SpecEndpoint) u16 {
    if (ep.node) |ni| {
        const node = &plan.nodes[ni];
        const sig = node.component.signature();
        return plan.n_external_slots + node.out_offset +
            contract.slotsBefore(sig.outputs, ep.index);
    }
    return plan.external_offsets[ep.index];
}

fn generatedName(n: u16, diag: *Diagnostic) CompileError!base.Name {
    var buf: [limits.max_name_len]u8 = undefined;
    const text = std.fmt.bufPrint(&buf, "__x_{d}", .{n}) catch {
        diag.set("cannot name generated node {d}", .{n});
        return error.TooManyGeneratedNodes;
    };
    return base.Name.init(text) catch {
        diag.set("generated node name '{s}' is invalid", .{text});
        return error.TooManyGeneratedNodes;
    };
}

// ---------------------------------------------------------------------------------------
// Drivers, links and order.
// ---------------------------------------------------------------------------------------

/// Counts the sources of every input port using the (zeroed) wiring table, then rejects a
/// `single` port with zero or several sources. `layoutLinks` overwrites the table afterwards.
fn checkDrivers(plan: *Plan, resolved: []const Resolved, diag: *Diagnostic) CompileError!void {
    for (plan.nodes) |*node| node.inputs = @splat(.{});
    for (resolved) |r| plan.nodes[r.to_node].inputs[r.to_port].count += 1;

    for (plan.nodes) |*node| {
        const sig = node.component.signature();
        for (sig.inputs, 0..) |port, pi| {
            const count = node.inputs[pi].count;
            if (port.ptype.fan_in == .sum) continue;
            if (count == 0) {
                diag.set("input '{s}.{s}' is not connected", .{ node.name.slice(), port.name });
                return error.UnconnectedInput;
            }
            if (count > 1) {
                diag.set("input '{s}.{s}' is driven {d} times (only summing ports take several)", .{
                    node.name.slice(), port.name, count,
                });
                return error.MultiplyDriven;
            }
        }
    }
}

fn layoutLinks(plan: *Plan, resolved: []const Resolved) void {
    var cursor: u16 = 0;
    for (plan.nodes, 0..) |*node, ni| {
        const sig = node.component.signature();
        var slot: u16 = 0;
        for (sig.inputs, 0..) |port, pi| {
            const first = cursor;
            for (resolved) |r| {
                if (r.to_node != ni or r.to_port != pi) continue;
                plan.links[cursor] = .{ .source = r.source, .gain = r.gain };
                cursor += 1;
            }
            node.inputs[pi] = .{
                .first = first,
                .count = cursor - first,
                .slot = slot,
                .width = port.ptype.width(),
            };
            slot += @as(u16, port.ptype.width());
        }
        assert(slot == @as(u16, node.n_in));
    }
    assert(cursor == plan.n_links);
}

/// Kahn's algorithm. Only links into feedthrough components constrain the order: a component
/// without feedthrough computes its outputs from its states alone.
fn orderNodes(
    plan: *Plan,
    resolved: []const Resolved,
    waiting: []u16,
    placed: []bool,
    diag: *Diagnostic,
) CompileError!void {
    const n: usize = plan.n_nodes;
    assert(waiting.len == n and placed.len == n);
    @memset(waiting, 0);
    @memset(placed, false);

    for (resolved) |r| {
        if (constrains(plan, r)) waiting[r.to_node] += 1;
    }
    for (0..n) |step| {
        const next = firstReady(n, waiting, placed) orelse {
            reportLoop(plan, placed, diag);
            return error.AlgebraicLoop;
        };
        placed[next] = true;
        plan.order[step] = @intCast(next);
        for (resolved) |r| {
            if (r.from_node == @as(u16, @intCast(next)) and constrains(plan, r)) {
                waiting[r.to_node] -= 1;
            }
        }
    }
}

fn constrains(plan: *const Plan, r: Resolved) bool {
    return r.from_node != null and plan.nodes[r.to_node].component.signature().feedthrough;
}

fn firstReady(n: usize, waiting: []const u16, placed: []const bool) ?usize {
    for (0..n) |i| {
        if (!placed[i] and waiting[i] == 0) return i;
    }
    return null;
}

fn reportLoop(plan: *const Plan, placed: []const bool, diag: *Diagnostic) void {
    var buf: [200]u8 = undefined;
    var w = std.Io.Writer.fixed(&buf);
    for (plan.nodes, 0..) |*node, i| {
        if (placed[i]) continue;
        w.print("{s}{s}", .{ if (w.end == 0) "" else ", ", node.name.slice() }) catch break;
    }
    diag.set("algebraic loop among components: {s}", .{w.buffered()});
}

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

const ComponentSpec = spec_mod.ComponentSpec;
const ConnectionSpec = spec_mod.ConnectionSpec;
const ExternalSpec = spec_mod.ExternalSpec;

const test_plan_bytes = 256 * 1024;
const test_ws_bytes = 8 * 1024;

const gain2: spec_mod.Component = .{ .gain = .{ .linear = .{ .k = 2.0 } } };
const mass1: spec_mod.Component = .{ .point_mass = .{ .rigid = .{ .mass_kg = 1.0 } } };

fn compileForTest(
    spec: *const Spec,
    storage: []align(plan_mod.storage_alignment) u8,
    diag: *Diagnostic,
) !*Plan {
    try std.testing.expect(bytesNeeded(spec) <= storage.len);
    return compileInto(storage, spec, diag);
}

fn expectFails(spec: Spec, expected: CompileError) !void {
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    try std.testing.expectError(expected, compileForTest(&spec, &storage, &diag));
    try std.testing.expect(diag.message().len > 0);
}

test "a chain of gains compiles in dependency order" {
    // Declared out of order on purpose: c depends on b depends on a.
    const spec: Spec = .{
        .components = &.{
            .{ .name = "c", .component = gain2 },
            .{ .name = "b", .component = gain2 },
            .{ .name = "a", .component = gain2 },
        },
        .externals = &.{.{ .name = "x", .dim = base.dim.dimensionless }},
        .connections = &.{
            .{ .from = "x", .to = "a.x" },
            .{ .from = "a.y", .to = "b.x" },
            .{ .from = "b.y", .to = "c.x" },
        },
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    try std.testing.expectEqual(@as(u16, 3), plan.n_nodes);
    const names = [_][]const u8{
        plan.nodes[plan.order[0]].name.slice(),
        plan.nodes[plan.order[1]].name.slice(),
        plan.nodes[plan.order[2]].name.slice(),
    };
    try std.testing.expectEqualStrings("a", names[0]);
    try std.testing.expectEqualStrings("b", names[1]);
    try std.testing.expectEqualStrings("c", names[2]);

    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    const ws = plan_mod.Workspace.into(plan, &ws_storage);
    const v = plan.evaluate(&.{}, &.{3.0}, ws);
    try std.testing.expectEqual(base.Validity.valid, v);
    try std.testing.expectEqual(@as(f64, 24.0), ws.signals[plan.signalIndex("c.y").?]);
    try std.testing.expectEqual(@as(f64, 6.0), ws.signals[plan.signalIndex("a.y").?]);
}

test "structural errors are caught with a diagnostic" {
    const x = ExternalSpec{ .name = "x", .dim = base.dim.dimensionless };
    const wired_a = [_]ConnectionSpec{.{ .from = "x", .to = "a.x" }};

    const bad_name: [1]ComponentSpec = .{.{ .name = "bad name", .component = gain2 }};
    try expectFails(.{ .components = &bad_name }, error.InvalidName);
    try expectFails(.{ .components = &.{
        .{ .name = "a", .component = gain2 },
        .{ .name = "a", .component = gain2 },
    } }, error.DuplicateName);
    try expectFails(.{
        .components = &.{.{ .name = "a", .component = gain2 }},
        .externals = &.{ x, x },
    }, error.DuplicateName);
    const lone_gain: [1]ComponentSpec = .{.{ .name = "a", .component = gain2 }};
    try expectFails(.{ .components = &lone_gain }, error.UnconnectedInput);
    try expectFails(.{
        .components = &.{.{ .name = "a", .component = gain2 }},
        .externals = &.{x},
        .connections = &(wired_a ++ wired_a),
    }, error.MultiplyDriven);
    try expectFails(.{
        .components = &.{.{ .name = "a", .component = gain2 }},
        .externals = &.{x},
        .connections = &.{.{ .from = "nope", .to = "a.x" }},
    }, error.UnknownExternal);
    try expectFails(.{
        .components = &.{.{ .name = "a", .component = gain2 }},
        .externals = &.{x},
        .connections = &.{.{ .from = "x", .to = "ghost.x" }},
    }, error.UnknownComponent);
    try expectFails(.{
        .components = &.{.{ .name = "a", .component = gain2 }},
        .externals = &.{x},
        .connections = &.{.{ .from = "x", .to = "a.z" }},
    }, error.UnknownPort);
    try expectFails(.{
        .components = &.{.{ .name = "a", .component = gain2 }},
        .externals = &.{x},
        .connections = &.{.{ .from = "x", .to = "a" }},
    }, error.BadPath);
    try expectFails(.{
        .components = &.{.{ .name = "a", .component = gain2 }},
        .externals = &.{x},
        .connections = &.{.{ .from = "x", .to = "a.x", .gain = std.math.nan(f64) }},
    }, error.InvalidGain);
}

test "connecting mismatched dimensions is rejected and explained" {
    const spec: Spec = .{
        .components = &.{.{ .name = "body", .component = mass1 }},
        .externals = &.{.{ .name = "speed", .dim = base.dim.speed }},
        .connections = &.{.{ .from = "speed", .to = "body.force" }},
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    try std.testing.expectError(
        error.DimensionMismatch,
        compileForTest(&spec, &storage, &diag),
    );
    try std.testing.expectEqualStrings(
        "speed [m*s^-1] cannot drive body.force [m*kg*s^-2]",
        diag.message(),
    );
}

test "bad parameters and initial states are rejected" {
    const bad_mass: spec_mod.Component = .{ .point_mass = .{ .rigid = .{ .mass_kg = -1.0 } } };
    const bad: [1]ComponentSpec = .{.{ .name = "m", .component = bad_mass }};
    try expectFails(.{ .components = &bad }, error.InvalidParameter);
    try expectFails(.{ .components = &.{
        .{ .name = "m", .component = mass1, .initial_state = &.{1.0} },
    } }, error.BadInitialState);
    try expectFails(.{ .components = &.{
        .{ .name = "m", .component = mass1, .initial_state = &.{ 1.0, std.math.nan(f64) } },
    } }, error.BadInitialState);
}

test "an algebraic loop is detected and named" {
    const spec: Spec = .{
        .components = &.{
            .{ .name = "a", .component = gain2 },
            .{ .name = "b", .component = gain2 },
        },
        .connections = &.{
            .{ .from = "a.y", .to = "b.x" },
            .{ .from = "b.y", .to = "a.x" },
        },
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    try std.testing.expectError(error.AlgebraicLoop, compileForTest(&spec, &storage, &diag));
    try std.testing.expectEqualStrings("algebraic loop among components: a, b", diag.message());

    // A component feeding itself is the smallest loop.
    const selfie: Spec = .{
        .components = &.{.{ .name = "a", .component = gain2 }},
        .connections = &.{.{ .from = "a.y", .to = "a.x" }},
    };
    try std.testing.expectError(error.AlgebraicLoop, compileForTest(&selfie, &storage, &diag));
}

test "the old component cap is gone and an undersized buffer is refused" {
    // 65 summing nodes: more than the old `max_components`, and each `force` input may be
    // legally unconnected.
    var comps: [65]ComponentSpec = undefined;
    var names: [65][8]u8 = undefined;
    for (&comps, 0..) |*c, i| {
        const name = std.fmt.bufPrint(&names[i], "m{d}", .{i}) catch unreachable;
        c.* = .{ .name = name, .component = mass1 };
    }
    const spec: Spec = .{ .components = &comps };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    try std.testing.expectEqual(@as(u16, 65), plan.n_nodes);

    var small: [8]u8 align(plan_mod.storage_alignment) = undefined;
    var small_diag: Diagnostic = .{};
    try std.testing.expectError(error.BufferTooSmall, compileInto(&small, &spec, &small_diag));
    try std.testing.expect(small_diag.message().len > 0);
}

test "summing ports accept several sources and none" {
    const spec: Spec = .{
        .components = &.{.{ .name = "mass", .component = mass1 }},
        .externals = &.{
            .{ .name = "f1", .dim = base.dim.force },
            .{ .name = "f2", .dim = base.dim.force },
        },
        .connections = &.{
            .{ .from = "f1", .to = "mass.force" },
            .{ .from = "f2", .to = "mass.force", .gain = -1.0 },
        },
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    const ws = plan_mod.Workspace.into(plan, &ws_storage);
    var dx: [2]f64 = undefined;
    _ = plan.rhs(&.{ 0.0, 0.0 }, &.{ 10.0, 4.0 }, ws, &dx);
    try std.testing.expectEqual(@as(f64, 6.0), dx[0]); // (10 - 4) / 1 kg

    const unloaded: Spec = .{ .components = &.{.{ .name = "mass", .component = mass1 }} };
    const plan2 = try compileForTest(&unloaded, &storage, &diag);
    const ws2 = plan_mod.Workspace.into(plan2, &ws_storage);
    _ = plan2.rhs(&.{ 0.0, 0.0 }, &.{}, ws2, &dx);
    try std.testing.expectEqual(@as(f64, 0.0), dx[0]);
}

test "initial states land in the right place" {
    const spec: Spec = .{ .components = &.{
        .{ .name = "m1", .component = mass1, .initial_state = &.{ 1.5, 2.5 } },
        .{ .name = "m2", .component = mass1 },
        .{ .name = "m3", .component = mass1, .initial_state = &.{ -1.0, 9.0 } },
    } };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    try std.testing.expectEqual(@as(u16, 6), plan.n_states);
    const expected = [_]f64{ 1.5, 2.5, 0.0, 0.0, -1.0, 9.0 };
    try std.testing.expectEqualSlices(f64, &expected, plan.initial_state[0..6]);
    try std.testing.expectEqual(@as(?u16, 5), plan.stateIndex("m3.position"));
    try std.testing.expectEqual(@as(?u16, 2), plan.signalIndex("m2.velocity"));
}

/// One gain wired to one external: no states, so the workspace is header plus two slices.
const gain_spec: Spec = .{
    .components = &.{.{ .name = "a", .component = gain2 }},
    .externals = &.{.{ .name = "x", .dim = base.dim.dimensionless }},
    .connections = &.{.{ .from = "x", .to = "a.x" }},
};

test "bytesNeeded is exact and one byte less is refused" {
    const needed = comptime bytesNeeded(&gain_spec);
    var exact: [needed]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileInto(&exact, &gain_spec, &diag);

    // Every slice is exactly as long as its count, and the header sits at offset 0.
    try std.testing.expectEqual(@as(usize, 1), plan.external_names.len);
    try std.testing.expectEqual(@as(usize, 1), plan.nodes.len);
    try std.testing.expectEqual(@as(usize, 1), plan.order.len);
    try std.testing.expectEqual(@as(usize, 1), plan.links.len);
    try std.testing.expectEqual(@as(usize, 2), plan.n_signals);
    try std.testing.expectEqual(@as(usize, 0), plan.n_states);
    // The header sits at offset 0 of the caller's buffer.
    try std.testing.expectEqual(@intFromPtr(&exact), @intFromPtr(plan));

    var short_diag: Diagnostic = .{};
    try std.testing.expectError(
        error.BufferTooSmall,
        compileInto(exact[0 .. needed - 1], &gain_spec, &short_diag),
    );
    try std.testing.expect(short_diag.message().len > 0);

    // A zero-state workspace still carries its header, signals and validity slices.
    const ws_bytes = plan_mod.Workspace.bytesNeeded(plan);
    try std.testing.expect(ws_bytes >= @sizeOf(plan_mod.Workspace));
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    const ws = plan_mod.Workspace.into(plan, &ws_storage);
    try std.testing.expectEqual(@as(usize, 2), ws.signals.len);
    try std.testing.expectEqual(@as(usize, 1), ws.validity.len);
    _ = plan.evaluate(&.{}, &.{3.0}, ws);
    try std.testing.expectEqual(@as(f64, 6.0), ws.signals[1]);
}

test "the same spec in two buffers gives identical plans and signals" {
    const needed = comptime bytesNeeded(&gain_spec);
    var buffer_a: [needed]u8 align(plan_mod.storage_alignment) = undefined;
    var buffer_b: [needed]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan_a = try compileInto(&buffer_a, &gain_spec, &diag);
    const plan_b = try compileInto(&buffer_b, &gain_spec, &diag);

    try std.testing.expectEqualSlices(u16, plan_a.order, plan_b.order);
    try std.testing.expectEqual(plan_a.n_links, plan_b.n_links);
    try std.testing.expectEqualSlices(f64, plan_a.initial_state, plan_b.initial_state);

    var ws_a_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_b_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    const ws_a = plan_mod.Workspace.into(plan_a, &ws_a_storage);
    const ws_b = plan_mod.Workspace.into(plan_b, &ws_b_storage);
    _ = plan_a.evaluate(&.{}, &.{1.5}, ws_a);
    _ = plan_b.evaluate(&.{}, &.{1.5}, ws_b);
    try std.testing.expectEqualSlices(f64, ws_a.signals, ws_b.signals);
}

test "one buffer can hold plan A, then plan B" {
    const spec_b: Spec = .{
        .components = &.{.{ .name = "b", .component = gain2 }},
        .externals = &.{.{ .name = "y", .dim = base.dim.dimensionless }},
        .connections = &.{.{ .from = "y", .to = "b.x" }},
    };
    var buffer: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};

    const plan_a = try compileForTest(&gain_spec, &buffer, &diag);
    const ws_a = plan_mod.Workspace.into(plan_a, &ws_storage);
    _ = plan_a.evaluate(&.{}, &.{2.0}, ws_a);
    const value_a = ws_a.signals[plan_a.signalIndex("a.y").?];

    // Recompiling overwrites `buffer`; `plan_a` is dead from here on.
    const plan_b = try compileForTest(&spec_b, &buffer, &diag);
    try std.testing.expectEqual(@as(f64, 4.0), value_a);
    const ws_b = plan_mod.Workspace.into(plan_b, &ws_storage);
    _ = plan_b.evaluate(&.{}, &.{5.0}, ws_b);
    try std.testing.expectEqual(@as(f64, 10.0), ws_b.signals[plan_b.signalIndex("b.y").?]);
}

const body_static: spec_mod.Component = .{ .body = .{ .static = .{} } };
const load_point: spec_mod.Component = .{ .load = .{ .point = .{} } };
const world_force: ExternalSpec = .{
    .name = "f",
    .dim = base.dim.force,
    .shape = .vec3,
    .quantity = .force,
    .frame = .world,
};

fn vecAt(ws: *const plan_mod.Workspace, slots: plan_mod.Plan.SignalSlots) base.Vec3 {
    return .{
        .x = ws.signals[slots.index],
        .y = ws.signals[slots.index + 1],
        .z = ws.signals[slots.index + 2],
    };
}

fn expectVec(expected: base.Vec3, actual: base.Vec3) !void {
    try std.testing.expectApproxEqAbs(expected.x, actual.x, 1.0e-12);
    try std.testing.expectApproxEqAbs(expected.y, actual.y, 1.0e-12);
    try std.testing.expectApproxEqAbs(expected.z, actual.z, 1.0e-12);
}

test "a world vector reaches an identity local port unchanged" {
    const spec: Spec = .{
        .components = &.{
            .{ .name = "body", .component = body_static },
            .{ .name = "l", .component = load_point },
        },
        .externals = &.{world_force},
        .connections = &.{.{ .from = "f", .to = "l.force" }},
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    try std.testing.expectEqual(@as(u16, 3), plan.n_nodes);
    try std.testing.expectEqual(@as(u16, 3), plan.n_links);
    try std.testing.expectEqual(@as(u16, 3), plan.n_external_slots);

    const ws = plan_mod.Workspace.into(plan, &ws_storage);
    _ = plan.evaluate(plan.initial_state, &.{ 1.0, 2.0, 3.0 }, ws);
    try expectVec(.{ .x = 1.0, .y = 2.0, .z = 3.0 }, vecAt(ws, plan.signalSlots("l.force").?));
}

test "the body attitude rotates world into body" {
    // Yaw of 90 degrees: body +x maps to world +y, so world +x is body -y.
    const yaw = base.Quat.fromEuler(0.0, 0.0, std.math.pi / 2.0);
    const spec: Spec = .{
        .components = &.{
            .{
                .name = "body",
                .component = body_static,
                .initial_state = &.{ yaw.w, yaw.x, yaw.y, yaw.z },
            },
            .{ .name = "l", .component = load_point },
        },
        .externals = &.{world_force},
        .connections = &.{.{ .from = "f", .to = "l.force" }},
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    const ws = plan_mod.Workspace.into(plan, &ws_storage);
    _ = plan.evaluate(plan.initial_state, &.{ 1.0, 0.0, 0.0 }, ws);
    try expectVec(.{ .y = -1.0 }, vecAt(ws, plan.signalSlots("l.force").?));
}

test "world to local composes the placement with the body attitude" {
    // Level body, local frame yawed 90 degrees: world +x becomes local -y.
    const placement: spec_mod.Placement = .{ .attitude_rad = .{ .z = std.math.pi / 2.0 } };
    const spec: Spec = .{
        .components = &.{
            .{ .name = "body", .component = body_static },
            .{ .name = "l", .component = load_point, .placement = placement },
        },
        .externals = &.{world_force},
        .connections = &.{.{ .from = "f", .to = "l.force" }},
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    const ws = plan_mod.Workspace.into(plan, &ws_storage);
    _ = plan.evaluate(plan.initial_state, &.{ 1.0, 0.0, 0.0 }, ws);
    try expectVec(.{ .y = -1.0 }, vecAt(ws, plan.signalSlots("l.force").?));
}

test "a local vector rotates between two component frames" {
    const a_place: spec_mod.Placement = .{ .attitude_rad = .{ .z = 0.3 } };
    const b_place: spec_mod.Placement = .{ .attitude_rad = .{ .z = -0.5 } };
    const spec: Spec = .{
        .components = &.{
            .{ .name = "body", .component = body_static },
            .{ .name = "a", .component = load_point, .placement = a_place },
            .{ .name = "b", .component = load_point, .placement = b_place },
        },
        .externals = &.{.{
            .name = "f",
            .dim = base.dim.force,
            .shape = .vec3,
            .quantity = .force,
            .frame = .body,
        }},
        .connections = &.{
            .{ .from = "f", .to = "a.force" },
            .{ .from = "a.force", .to = "b.force" },
        },
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    const ws = plan_mod.Workspace.into(plan, &ws_storage);
    const input: base.Vec3 = .{ .x = 1.0, .y = 2.0, .z = 3.0 };
    _ = plan.evaluate(plan.initial_state, &.{ input.x, input.y, input.z }, ws);

    // f is body-frame, so a.force is already conj(q_a) * input; the A -> B node then gives
    // conj(q_b) * q_a * (conj(q_a) * input) = conj(q_b) * input.
    const qb = base.Quat.fromEuler(0.0, 0.0, -0.5);
    const expected = base.Quat.conjugate(qb).rotate(input);
    try expectVec(expected, vecAt(ws, plan.signalSlots("b.force").?));
}

test "a vec3 external occupies three slots" {
    const spec: Spec = .{
        .components = &.{
            .{ .name = "body", .component = body_static },
            .{ .name = "l", .component = load_point },
        },
        .externals = &.{world_force},
        .connections = &.{.{ .from = "f", .to = "l.force" }},
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    try std.testing.expectEqual(@as(u16, 3), plan.n_external_slots);
    try std.testing.expectEqual(@as(usize, 3), plan.external_ptypes[0].width());
    try std.testing.expectEqual(@as(u16, 0), plan.signalIndex("f").?);
    const slots = plan.signalSlots("f").?;
    try std.testing.expectEqual(@as(u16, 0), slots.index);
    try std.testing.expectEqual(@as(u8, 3), slots.width);
}

test "a transformed spec fits exactly in bytesNeeded" {
    const spec: Spec = .{
        .components = &.{
            .{ .name = "body", .component = body_static },
            .{ .name = "l", .component = load_point },
        },
        .externals = &.{world_force},
        .connections = &.{.{ .from = "f", .to = "l.force" }},
    };
    const needed = bytesNeeded(&spec);
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    _ = try compileInto(storage[0..needed], &spec, &diag);
    try std.testing.expectError(
        error.BufferTooSmall,
        compileInto(storage[0 .. needed - 1], &spec, &diag),
    );
}

test "a scalar with a frame is refused" {
    try expectFails(.{
        .components = &.{},
        .externals = &.{.{ .name = "x", .dim = base.dim.force, .frame = .body }},
    }, error.FrameMismatch);
}

test "a non-finite placement is refused" {
    try expectFails(.{ .components = &.{.{
        .name = "a",
        .component = gain2,
        .placement = .{ .position_m = .{ .x = std.math.nan(f64) } },
    }} }, error.BadPlacement);
}

test "two bodies are ambiguous" {
    try expectFails(.{ .components = &.{
        .{ .name = "b1", .component = body_static },
        .{ .name = "b2", .component = body_static },
    } }, error.AmbiguousBody);
}

test "the generated-node prefix is reserved" {
    try expectFails(.{
        .components = &.{.{ .name = "__x_0", .component = gain2 }},
    }, error.InvalidName);
}

test "an empty quaternion state defaults to the identity" {
    const spec: Spec = .{
        .components = &.{.{ .name = "body", .component = body_static }},
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    try std.testing.expectEqualSlices(f64, &.{ 1.0, 0.0, 0.0, 0.0 }, plan.initial_state);
    try std.testing.expectEqual(@as(usize, 1), plan.attitude_states.len);
    try std.testing.expectEqual(@as(u16, 0), plan.attitude_states[0]);
}

test "an initial quaternion is normalized at compile time" {
    const spec: Spec = .{
        .components = &.{.{
            .name = "body",
            .component = body_static,
            .initial_state = &.{ 2.0, 0.0, 0.0, 0.0 },
        }},
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    try std.testing.expectEqualSlices(f64, &.{ 1.0, 0.0, 0.0, 0.0 }, plan.initial_state);
}

test "a zero initial quaternion is refused" {
    try expectFails(.{
        .components = &.{.{
            .name = "body",
            .component = body_static,
            .initial_state = &.{ 0.0, 0.0, 0.0, 0.0 },
        }},
    }, error.BadInitialState);
}

test "the generated-node prefix is reserved for externals too" {
    try expectFails(.{
        .components = &.{},
        .externals = &.{.{ .name = "__x_0", .dim = base.dim.dimensionless }},
    }, error.InvalidName);
}

fn wrenchAt(ws: *const plan_mod.Workspace, slots: plan_mod.Plan.SignalSlots) base.Wrench {
    return .{
        .force = vecAt(ws, slots),
        .moment = .{
            .x = ws.signals[slots.index + 3],
            .y = ws.signals[slots.index + 4],
            .z = ws.signals[slots.index + 5],
        },
    };
}

test "a rigid wrench at an offset produces body force and moment" {
    // A +x force 0.4 m above the CG (z is down) pitches the body nose down: M = r x F = -y.
    const place: spec_mod.Placement = .{ .position_m = .{ .z = -0.4 } };
    const spec: Spec = .{
        .components = &.{
            .{ .name = "body", .component = body_static },
            .{ .name = "l", .component = load_point, .placement = place },
        },
        .externals = &.{world_force},
        .connections = &.{
            .{ .from = "f", .to = "l.force" },
            .{ .from = "l.wrench", .to = "body.mount", .attachment = .rigid },
        },
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);

    // One rotate for f -> l.force, one mount for l.wrench -> body.mount.
    try std.testing.expectEqual(@as(u16, 4), plan.n_nodes);
    const mount_slots = plan.signalSlots("__x_1.wrench").?;
    const body_mount = plan.nodes[0].inputs[0];
    try std.testing.expectEqual(@as(u16, 1), body_mount.count);
    try std.testing.expectEqual(mount_slots.index, plan.links[body_mount.first].source);

    const ws = plan_mod.Workspace.into(plan, &ws_storage);
    _ = plan.evaluate(plan.initial_state, &.{ 2.0, 0.0, 0.0 }, ws);
    const w = wrenchAt(ws, mount_slots);
    try expectVec(.{ .x = 2.0 }, w.force);
    try expectVec(.{ .y = -0.8 }, w.moment);
}

test "a component cannot be rigidly attached twice" {
    try expectFails(.{
        .components = &.{
            .{ .name = "body", .component = body_static },
            .{ .name = "l", .component = load_point },
        },
        .connections = &.{
            .{ .from = "l.wrench", .to = "body.mount", .attachment = .rigid },
            .{ .from = "l.wrench", .to = "body.mount", .attachment = .rigid },
        },
    }, error.MultiplyDriven);
}

test "a rigid attachment from a non-wrench output is refused" {
    try expectFails(.{
        .components = &.{
            .{ .name = "body", .component = body_static },
            .{ .name = "l", .component = load_point },
        },
        .connections = &.{
            .{ .from = "l.force", .to = "body.mount", .attachment = .rigid },
        },
    }, error.RigidAttachmentNotWrench);
}

test "a rigid attachment from an external is refused" {
    try expectFails(.{
        .components = &.{.{ .name = "body", .component = body_static }},
        .externals = &.{.{
            .name = "w",
            .dim = base.dim.force,
            .shape = .wrench,
            .quantity = .wrench,
            .frame = .world,
        }},
        .connections = &.{
            .{ .from = "w", .to = "body.mount", .attachment = .rigid },
        },
    }, error.RigidAttachmentNotWrench);
}

test "a rigid attachment to something other than the mount is refused" {
    try expectFails(.{
        .components = &.{
            .{ .name = "body", .component = body_static },
            .{ .name = "a", .component = load_point },
            .{ .name = "b", .component = load_point },
        },
        .connections = &.{
            .{ .from = "a.wrench", .to = "b.moment", .attachment = .rigid },
        },
    }, error.RigidAttachmentToNonBody);
}

test "a rigid attachment needs a body" {
    try expectFails(.{
        .components = &.{.{
            .name = "l",
            .component = load_point,
        }},
        .connections = &.{
            .{ .from = "l.wrench", .to = "l.moment", .attachment = .rigid },
        },
    }, error.AmbiguousBody);
}

test "a rigid spec fits exactly in bytesNeeded" {
    const place: spec_mod.Placement = .{ .position_m = .{ .z = -0.4 } };
    const spec: Spec = .{
        .components = &.{
            .{ .name = "body", .component = body_static },
            .{ .name = "l", .component = load_point, .placement = place },
        },
        .externals = &.{world_force},
        .connections = &.{
            .{ .from = "f", .to = "l.force" },
            .{ .from = "l.wrench", .to = "body.mount", .attachment = .rigid },
        },
    };
    const needed = bytesNeeded(&spec);
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    _ = try compileInto(storage[0..needed], &spec, &diag);
    try std.testing.expectError(
        error.BufferTooSmall,
        compileInto(storage[0 .. needed - 1], &spec, &diag),
    );
}

test "two components attach rigidly to the same mount" {
    const spec: Spec = .{
        .components = &.{
            .{ .name = "body", .component = body_static },
            .{
                .name = "a",
                .component = load_point,
                .placement = .{ .position_m = .{ .z = -0.4 } },
            },
            .{
                .name = "b",
                .component = load_point,
                .placement = .{ .position_m = .{ .x = 0.3 } },
            },
        },
        .connections = &.{
            .{ .from = "a.wrench", .to = "body.mount", .attachment = .rigid },
            .{ .from = "b.wrench", .to = "body.mount", .attachment = .rigid },
        },
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    try std.testing.expectEqual(@as(u16, 5), plan.n_nodes);
    const mount = plan.nodes[0].inputs[0];
    try std.testing.expectEqual(@as(u16, 2), mount.count);
    try std.testing.expectEqual(@as(u8, 6), mount.width);
    // Both links carry a wrench, from the two generated mount nodes.
    for (plan.links[mount.first..][0..mount.count]) |link| {
        try std.testing.expect(link.source >= plan.n_external_slots);
    }
}

const body_force: ExternalSpec = .{
    .name = "fb",
    .dim = base.dim.force,
    .shape = .vec3,
    .quantity = .force,
    .frame = .body,
};

test "a rigid mount uses the source's orientation and offset" {
    // Local +x is yawed 90 degrees, so it points along body +y. The offset is 0.4 m above
    // the CG, so body M = r x F = +x.
    const place: spec_mod.Placement = .{
        .position_m = .{ .z = -0.4 },
        .attitude_rad = .{ .z = std.math.pi / 2.0 },
    };
    const spec: Spec = .{
        .components = &.{
            .{ .name = "body", .component = body_static },
            .{ .name = "l", .component = load_point, .placement = place },
        },
        .externals = &.{body_force},
        .connections = &.{
            .{ .from = "fb", .to = "l.force" },
            .{ .from = "l.wrench", .to = "body.mount", .attachment = .rigid },
        },
    };
    var storage: [test_plan_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var ws_storage: [test_ws_bytes]u8 align(plan_mod.storage_alignment) = undefined;
    var diag: Diagnostic = .{};
    const plan = try compileForTest(&spec, &storage, &diag);
    const ws = plan_mod.Workspace.into(plan, &ws_storage);
    _ = plan.evaluate(plan.initial_state, &.{ 0.0, 2.0, 0.0 }, ws);
    const w = wrenchAt(ws, plan.signalSlots("__x_1.wrench").?);
    try expectVec(.{ .y = 2.0 }, w.force);
    try expectVec(.{ .x = 0.8 }, w.moment);
}
