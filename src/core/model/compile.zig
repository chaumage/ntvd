//! Spec -> Plan. Rejects every structural mistake with a precise error and a diagnostic.
//!
//! Stages (each a small function, run in this order):
//!   1. counts        the architecture fits the static limits
//!   2. externals     names and dimensions of the boundary signals
//!   3. nodes         names, parameter checks, initial states, signal and state layout
//!   4. connections   paths resolve, dimensions agree, gains are finite
//!   5. drivers       every `single` input has exactly one source
//!   6. links         connections laid out per input port
//!   7. order         feedthrough components sorted after their sources
//!
//! Algebraic loops (feedthrough components feeding each other in a cycle) are rejected for
//! now. The way to break one is to make a state of it (a dynamic component) or to turn the
//! loop variable into an external that an analysis solves for.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const spec_mod = @import("spec.zig");
const plan_mod = @import("plan.zig");
const path = @import("path.zig");
const Diagnostic = @import("diagnostic.zig").Diagnostic;

const limits = base.limits;
const Spec = spec_mod.Spec;
const Plan = plan_mod.Plan;
const Node = plan_mod.Node;

pub const CompileError = error{
    TooManyComponents,
    TooManyExternals,
    TooManyConnections,
    InvalidName,
    DuplicateName,
    InvalidParameter,
    BadInitialState,
    BadPath,
    UnknownComponent,
    UnknownExternal,
    UnknownPort,
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

/// Compiles `spec` into `plan`. On error `plan` is unusable and `diag` explains why.
pub fn compile(spec: *const Spec, plan: *Plan, diag: *Diagnostic) CompileError!void {
    diag.* = .{};
    try checkCounts(spec, diag);
    try buildExternals(spec, plan, diag);
    try buildNodes(spec, plan, diag);

    var resolved: [limits.max_links]Resolved = undefined;
    const n = try resolveConnections(spec, plan, &resolved, diag);
    try checkDrivers(plan, resolved[0..n], diag);
    layoutLinks(plan, resolved[0..n]);
    try orderNodes(plan, resolved[0..n], diag);
}

fn checkCounts(spec: *const Spec, diag: *Diagnostic) CompileError!void {
    if (spec.components.len > limits.max_components) {
        diag.set("{d} components exceed the limit of {d}", .{
            spec.components.len, limits.max_components,
        });
        return error.TooManyComponents;
    }
    if (spec.externals.len > limits.max_externals) {
        diag.set("{d} externals exceed the limit of {d}", .{
            spec.externals.len, limits.max_externals,
        });
        return error.TooManyExternals;
    }
    if (spec.connections.len > limits.max_links) {
        diag.set("{d} connections exceed the limit of {d}", .{
            spec.connections.len, limits.max_links,
        });
        return error.TooManyConnections;
    }
}

fn makeName(text: []const u8, what: []const u8, diag: *Diagnostic) CompileError!base.Name {
    return base.Name.init(text) catch |err| {
        diag.set("invalid {s} name '{s}': {s}", .{ what, text, @errorName(err) });
        return error.InvalidName;
    };
}

fn buildExternals(spec: *const Spec, plan: *Plan, diag: *Diagnostic) CompileError!void {
    plan.n_externals = @intCast(spec.externals.len);
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

fn buildNodes(spec: *const Spec, plan: *Plan, diag: *Diagnostic) CompileError!void {
    plan.n_nodes = @intCast(spec.components.len);
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
        node.name = name;
        node.component = cs.component;
        node.n_in = @intCast(sig.inputs.len);
        node.n_out = @intCast(sig.outputs.len);
        node.n_state = @intCast(sig.states.len);
        node.state_offset = state_cursor;
        node.out_offset = out_cursor;
        node.inputs = @splat(.{});

        try copyInitialState(cs, plan, state_cursor, diag);
        state_cursor += node.n_state;
        out_cursor += node.n_out;
    }
    plan.n_states = state_cursor;
    plan.n_signals = plan.n_externals + out_cursor;
    assert(plan.n_states <= limits.max_states and plan.n_signals <= limits.max_signals);
}

fn copyInitialState(
    cs: spec_mod.ComponentSpec,
    plan: *Plan,
    offset: u16,
    diag: *Diagnostic,
) CompileError!void {
    const n = cs.component.signature().states.len;
    const dest = plan.initial_state[offset..][0..n];
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
    out: *[limits.max_links]Resolved,
    diag: *Diagnostic,
) CompileError!u32 {
    for (spec.connections, 0..) |c, i| {
        out[i] = try resolveOne(c, plan, diag);
    }
    return @intCast(spec.connections.len);
}

const Endpoint = struct { signal_or_port: u16, node: ?u16, dim: base.Dim };

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
    if (!from.dim.eql(to.dim)) {
        diag.set("{s} [{f}] cannot drive {s} [{f}]", .{ c.from, from.dim, c.to, to.dim });
        return error.DimensionMismatch;
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
        return .{ .signal_or_port = ei, .node = null, .dim = plan.external_dims[ei] };
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
    return .{
        .signal_or_port = plan.n_externals + node.out_offset + @as(u16, @intCast(oi)),
        .node = ni,
        .dim = sig.outputs[oi].dim,
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
    return .{ .signal_or_port = @intCast(ii), .node = ni, .dim = sig.inputs[ii].dim };
}

fn checkDrivers(
    plan: *const Plan,
    resolved: []const Resolved,
    diag: *Diagnostic,
) CompileError!void {
    var counts: [limits.max_components][limits.max_ports]u16 = @splat(@splat(0));
    for (resolved) |r| counts[r.to_node][r.to_port] += 1;

    for (plan.nodes[0..plan.n_nodes], 0..) |*node, ni| {
        const sig = node.component.signature();
        for (sig.inputs, 0..) |port, pi| {
            const count = counts[ni][pi];
            if (port.fan_in == .sum) continue;
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
    for (plan.nodes[0..plan.n_nodes], 0..) |*node, ni| {
        for (0..node.n_in) |pi| {
            const first = cursor;
            for (resolved) |r| {
                if (r.to_node != ni or r.to_port != pi) continue;
                plan.links[cursor] = .{ .source = r.source, .gain = r.gain };
                cursor += 1;
            }
            node.inputs[pi] = .{ .first = first, .count = cursor - first };
        }
    }
    plan.n_links = cursor;
    assert(cursor == resolved.len);
}

/// Kahn's algorithm. Only links into feedthrough components constrain the order: a component
/// without feedthrough computes its outputs from its states alone.
fn orderNodes(plan: *Plan, resolved: []const Resolved, diag: *Diagnostic) CompileError!void {
    const n = plan.n_nodes;
    var waiting: Counters = @splat(0);
    var placed: Flags = @splat(false);

    for (resolved) |r| {
        if (constrains(plan, r)) waiting[r.to_node] += 1;
    }
    for (0..n) |step| {
        const next = firstReady(n, &waiting, &placed) orelse {
            reportLoop(plan, &placed, diag);
            return error.AlgebraicLoop;
        };
        placed[next] = true;
        plan.order[step] = next;
        for (resolved) |r| {
            if (r.from_node == next and constrains(plan, r)) waiting[r.to_node] -= 1;
        }
    }
}

fn constrains(plan: *const Plan, r: Resolved) bool {
    return r.from_node != null and plan.nodes[r.to_node].component.signature().feedthrough;
}

const Counters = [limits.max_components]u16;
const Flags = [limits.max_components]bool;

fn firstReady(n: u16, waiting: *const Counters, placed: *const Flags) ?u16 {
    for (0..n) |i| {
        if (!placed[i] and waiting[i] == 0) return @intCast(i);
    }
    return null;
}

fn reportLoop(plan: *const Plan, placed: *const Flags, diag: *Diagnostic) void {
    var buf: [200]u8 = undefined;
    var w = std.Io.Writer.fixed(&buf);
    for (plan.nodes[0..plan.n_nodes], 0..) |*node, i| {
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

const gain2: spec_mod.Component = .{ .gain = .{ .linear = .{ .k = 2.0 } } };
const mass1: spec_mod.Component = .{ .point_mass = .{ .rigid = .{ .mass_kg = 1.0 } } };

fn expectFails(spec: Spec, expected: CompileError) !void {
    var plan: Plan = undefined;
    var diag: Diagnostic = .{};
    try std.testing.expectError(expected, compile(&spec, &plan, &diag));
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
    var plan: Plan = undefined;
    var diag: Diagnostic = .{};
    try compile(&spec, &plan, &diag);
    try std.testing.expectEqual(@as(u16, 3), plan.n_nodes);
    const names = [_][]const u8{
        plan.nodes[plan.order[0]].name.slice(),
        plan.nodes[plan.order[1]].name.slice(),
        plan.nodes[plan.order[2]].name.slice(),
    };
    try std.testing.expectEqualStrings("a", names[0]);
    try std.testing.expectEqualStrings("b", names[1]);
    try std.testing.expectEqualStrings("c", names[2]);

    var ws: plan_mod.Workspace = .{};
    const v = plan.evaluate(&.{}, &.{3.0}, &ws);
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
    var plan: Plan = undefined;
    var diag: Diagnostic = .{};
    try std.testing.expectError(error.DimensionMismatch, compile(&spec, &plan, &diag));
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
    var plan: Plan = undefined;
    var diag: Diagnostic = .{};
    try std.testing.expectError(error.AlgebraicLoop, compile(&spec, &plan, &diag));
    try std.testing.expectEqualStrings("algebraic loop among components: a, b", diag.message());

    // A component feeding itself is the smallest loop.
    const selfie: Spec = .{
        .components = &.{.{ .name = "a", .component = gain2 }},
        .connections = &.{.{ .from = "a.y", .to = "a.x" }},
    };
    try std.testing.expectError(error.AlgebraicLoop, compile(&selfie, &plan, &diag));
}

test "limits are enforced before anything is built" {
    var many: [limits.max_components + 1]ComponentSpec = undefined;
    for (&many, 0..) |*c, i| {
        _ = i;
        c.* = .{ .name = "g", .component = gain2 };
    }
    try expectFails(.{ .components = &many }, error.TooManyComponents);
    var externals: [limits.max_externals + 1]ExternalSpec = undefined;
    for (&externals) |*e| e.* = .{ .name = "e", .dim = base.dim.length };
    try expectFails(.{ .components = &.{}, .externals = &externals }, error.TooManyExternals);
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
    var plan: Plan = undefined;
    var diag: Diagnostic = .{};
    try compile(&spec, &plan, &diag);
    var ws: plan_mod.Workspace = .{};
    var dx: [2]f64 = undefined;
    _ = plan.rhs(&.{ 0.0, 0.0 }, &.{ 10.0, 4.0 }, &ws, &dx);
    try std.testing.expectEqual(@as(f64, 6.0), dx[0]); // (10 - 4) / 1 kg

    const unloaded: Spec = .{ .components = &.{.{ .name = "mass", .component = mass1 }} };
    try compile(&unloaded, &plan, &diag);
    _ = plan.rhs(&.{ 0.0, 0.0 }, &.{}, &ws, &dx);
    try std.testing.expectEqual(@as(f64, 0.0), dx[0]);
}

test "initial states land in the right place" {
    const spec: Spec = .{ .components = &.{
        .{ .name = "m1", .component = mass1, .initial_state = &.{ 1.5, 2.5 } },
        .{ .name = "m2", .component = mass1 },
        .{ .name = "m3", .component = mass1, .initial_state = &.{ -1.0, 9.0 } },
    } };
    var plan: Plan = undefined;
    var diag: Diagnostic = .{};
    try compile(&spec, &plan, &diag);
    try std.testing.expectEqual(@as(u16, 6), plan.n_states);
    const expected = [_]f64{ 1.5, 2.5, 0.0, 0.0, -1.0, 9.0 };
    try std.testing.expectEqualSlices(f64, &expected, plan.initial_state[0..6]);
    try std.testing.expectEqual(@as(?u16, 5), plan.stateIndex("m3.position"));
    try std.testing.expectEqual(@as(?u16, 2), plan.signalIndex("m2.velocity"));
}
