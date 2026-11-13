//! Spec -> Plan. Rejects every structural mistake with a precise error and a diagnostic.
//!
//! Stages (each a small function, run in this order):
//!   1. measure       exact layout and byte count of the plan
//!   2. build         externals, nodes, initial states, signal and state layout
//!   3. connections   paths resolve, dimensions agree, gains are finite
//!   4. drivers       every `single` input has exactly one source
//!   5. links         connections laid out per input port
//!   6. order         feedthrough components sorted after their sources
//!
//! The plan is carved from a caller-owned buffer. `bytesNeeded` and `compileInto` share
//! `measure`, so the required size and the actual layout cannot drift.
//!
//! Algebraic loops (feedthrough components feeding each other in a cycle) are rejected for
//! now. The way to break one is to make a state of it (a dynamic component) or to turn the
//! loop variable into an external that an analysis solves for.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const components_contract = @import("components").contract;
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
const Dim = base.Dim;
const Sizer = storage_mod.Sizer;
const Carver = storage_mod.Carver;

pub const CompileError = error{
    BufferTooSmall,
    TooManyComponents,
    TooManyExternals,
    TooManyConnections,
    TooManyStates,
    TooManySignals,
    InvalidName,
    DuplicateName,
    InvalidParameter,
    BadInitialState,
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
};

/// A connection with all names resolved to indices.
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
    if (@alignOf(Dim) > storage_mod.alignment) {
        @compileError("Dim alignment exceeds storage");
    }
}

/// Counts and byte size of one plan. `usize` so the size can be computed before any ceiling
/// check; `compileInto` rejects anything that does not fit the `u16` encoding.
pub const Layout = struct {
    n_externals: usize,
    n_nodes: usize,
    n_links: usize,
    n_states: usize,
    n_signals: usize,
    n_attitude_states: usize,
    n_connections: usize,
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
    const external_dims = carver.take(Dim, layout.n_externals);
    const nodes = carver.take(Node, layout.n_nodes);
    const order = carver.take(u16, layout.n_nodes);
    const links = carver.take(Link, layout.n_links);
    const initial_state = carver.take(f64, layout.n_states);
    const attitude_states = carver.take(u16, layout.n_attitude_states);
    const resolved = carver.take(Resolved, layout.n_connections);
    const waiting = carver.take(u16, layout.n_nodes);
    const placed = carver.take(bool, layout.n_nodes);
    assert(carver.cursor == layout.bytes);

    plan.* = .{
        .external_names = external_names,
        .external_dims = external_dims,
        .nodes = nodes,
        .order = order,
        .links = links,
        .initial_state = initial_state,
        .attitude_states = attitude_states,
        .n_externals = @intCast(layout.n_externals),
        .n_nodes = @intCast(layout.n_nodes),
        .n_links = @intCast(layout.n_links),
        .n_states = @intCast(layout.n_states),
        .n_signals = @intCast(layout.n_signals),
    };

    try buildExternals(spec, plan, diag);
    try buildNodes(spec, plan, initial_state, diag);
    const n = try resolveConnections(spec, plan, resolved, diag);
    try checkDrivers(plan, resolved[0..n], diag);
    layoutLinks(plan, resolved[0..n]);
    try orderNodes(plan, resolved[0..n], waiting, placed, diag);
    return plan;
}

/// Counts and byte size only. Never fails: an invalid spec still has a computable footprint.
fn measure(spec: *const Spec) Layout {
    var n_states: usize = 0;
    var n_outputs: usize = 0;
    for (spec.components) |cs| {
        const sig = cs.component.signature();
        n_states += @as(usize, sig.stateSlots());
        n_outputs += @as(usize, sig.outputSlots());
        // Phase 2b derives attitude slots from `quantity = .quaternion`; none yet.
    }
    const n_externals = spec.externals.len;
    const n_nodes = spec.components.len;
    const n_links = spec.connections.len;
    const n_signals = n_externals + n_outputs;
    const n_connections = spec.connections.len;

    var sizer: Sizer = .{};
    sizer.add(Plan, 1);
    sizer.add(Name, n_externals);
    sizer.add(Dim, n_externals);
    sizer.add(Node, n_nodes);
    sizer.add(u16, n_nodes);
    sizer.add(Link, n_links);
    sizer.add(f64, n_states);
    sizer.add(u16, 0);
    sizer.add(Resolved, n_connections);
    sizer.add(u16, n_nodes);
    sizer.add(bool, n_nodes);

    return .{
        .n_externals = n_externals,
        .n_nodes = n_nodes,
        .n_links = n_links,
        .n_states = n_states,
        .n_signals = n_signals,
        .n_attitude_states = 0,
        .n_connections = n_connections,
        .bytes = sizer.bytes(),
    };
}

fn checkCounts(layout: Layout, diag: *Diagnostic) CompileError!void {
    const ceiling: usize = limits.max_index;
    if (layout.n_externals > ceiling) {
        diag.set("{d} externals exceed the u16 encoding limit of {d}", .{
            layout.n_externals, ceiling,
        });
        return error.TooManyExternals;
    }
    if (layout.n_nodes > ceiling) {
        diag.set("{d} components exceed the u16 encoding limit of {d}", .{
            layout.n_nodes, ceiling,
        });
        return error.TooManyComponents;
    }
    if (layout.n_links > ceiling or layout.n_connections > ceiling) {
        diag.set("{d} connections exceed the u16 encoding limit of {d}", .{
            layout.n_connections, ceiling,
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
    return base.Name.init(text) catch |err| {
        diag.set("invalid {s} name '{s}': {s}", .{ what, text, @errorName(err) });
        return error.InvalidName;
    };
}

fn buildExternals(spec: *const Spec, plan: *Plan, diag: *Diagnostic) CompileError!void {
    for (spec.externals, 0..) |ext, i| {
        plan.external_names[i] = try makeName(ext.name, "external", diag);
        plan.external_dims[i] = ext.dim;
        for (spec.externals[0..i]) |other| {
            if (std.mem.eql(u8, other.name, ext.name)) {
                diag.set("external '{s}' is declared twice", .{ext.name});
                return error.DuplicateName;
            }
        }
    }
}

fn buildNodes(
    spec: *const Spec,
    plan: *Plan,
    initial_state: []f64,
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

        const sig = cs.component.signature();
        const node = &plan.nodes[i];
        const in_slots = sig.inputSlots();
        const out_slots = sig.outputSlots();
        const state_slots = sig.stateSlots();
        node.name = name;
        node.component = cs.component;
        node.n_in = @intCast(in_slots);
        node.n_out = @intCast(out_slots);
        node.n_state = @intCast(state_slots);
        node.state_offset = state_cursor;
        node.out_offset = out_cursor;
        node.inputs = @splat(.{});

        try copyInitialState(cs, initial_state, state_cursor, diag);
        state_cursor += state_slots;
        out_cursor += out_slots;
    }
    assert(state_cursor == plan.n_states);
    assert(plan.n_externals + out_cursor == plan.n_signals);
}

fn copyInitialState(
    cs: spec_mod.ComponentSpec,
    state: []f64,
    offset: u16,
    diag: *Diagnostic,
) CompileError!void {
    const n: usize = cs.component.signature().stateSlots();
    const dest = state[offset..][0..n];
    if (cs.initial_state.len == 0) {
        @memset(dest, 0.0);
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
}

fn resolveConnections(
    spec: *const Spec,
    plan: *const Plan,
    out: []Resolved,
    diag: *Diagnostic,
) CompileError!u32 {
    assert(out.len == spec.connections.len);
    for (spec.connections, 0..) |c, i| {
        out[i] = try resolveOne(c, plan, diag);
    }
    return @intCast(spec.connections.len);
}

const Endpoint = struct {
    signal_or_port: u16,
    node: ?u16,
    ptype: components_contract.PortType,
};

fn resolveOne(
    c: spec_mod.ConnectionSpec,
    plan: *const Plan,
    diag: *Diagnostic,
) CompileError!Resolved {
    if (!std.math.isFinite(c.gain)) {
        diag.set("connection {s} -> {s} has a non-finite gain", .{ c.from, c.to });
        return error.InvalidGain;
    }
    const from = try resolveSource(c.from, plan, diag);
    const to = try resolveTarget(c.to, plan, diag);
    if (components_contract.compatibility(from.ptype, to.ptype)) |reason| {
        if (reason == .dimension) {
            diag.set("{s} [{f}] cannot drive {s} [{f}]", .{
                c.from, from.ptype.dim, c.to, to.ptype.dim,
            });
            return error.DimensionMismatch;
        }
        diag.set("{s} cannot drive {s}: incompatible {s}", .{
            c.from, c.to, @tagName(reason),
        });
        return error.FrameMismatch;
    }
    return .{
        .to_node = to.node.?,
        .to_port = @intCast(to.signal_or_port),
        .source = from.signal_or_port,
        .from_node = from.node,
        .gain = c.gain,
    };
}

fn resolveSource(text: []const u8, plan: *const Plan, diag: *Diagnostic) CompileError!Endpoint {
    const p = path.parse(text) catch {
        diag.set("malformed source path '{s}'", .{text});
        return error.BadPath;
    };
    const port = p.port orelse {
        const ei = plan.externalIndex(p.head) orelse {
            diag.set("unknown external signal '{s}'", .{p.head});
            return error.UnknownExternal;
        };
        return .{
            .signal_or_port = ei,
            .node = null,
            .ptype = components_contract.scalar(plan.external_dims[ei]),
        };
    };
    const ni = plan.nodeIndex(p.head) orelse {
        diag.set("unknown component '{s}' in '{s}'", .{ p.head, text });
        return error.UnknownComponent;
    };
    const node = &plan.nodes[ni];
    const sig = node.component.signature();
    const oi = sig.outputIndex(port) orelse {
        diag.set("component '{s}' has no output '{s}'", .{ p.head, port });
        return error.UnknownPort;
    };
    const slot = components_contract.slotsBefore(sig.outputs, oi);
    return .{
        .signal_or_port = plan.n_externals + node.out_offset + slot,
        .node = ni,
        .ptype = sig.outputs[oi].ptype,
    };
}

fn resolveTarget(text: []const u8, plan: *const Plan, diag: *Diagnostic) CompileError!Endpoint {
    const p = path.parse(text) catch {
        diag.set("malformed target path '{s}'", .{text});
        return error.BadPath;
    };
    const port = p.port orelse {
        diag.set("target '{s}' must be written component.input", .{text});
        return error.BadPath;
    };
    const ni = plan.nodeIndex(p.head) orelse {
        diag.set("unknown component '{s}' in '{s}'", .{ p.head, text });
        return error.UnknownComponent;
    };
    const sig = plan.nodes[ni].component.signature();
    const ii = sig.inputIndex(port) orelse {
        diag.set("component '{s}' has no input '{s}'", .{ p.head, port });
        return error.UnknownPort;
    };
    return .{ .signal_or_port = @intCast(ii), .node = ni, .ptype = sig.inputs[ii].ptype };
}

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
