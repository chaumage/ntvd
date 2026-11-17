//! Architecture fuzzer: random valid and invalid specs against the model compiler.
//!
//! Contract under test, for every generated spec:
//!   * `bytesNeeded` yields a size, and `compileInto` given that much storage either returns
//!     a plan or an error with a diagnostic; it never crashes, hangs, or asserts on bad input.
//!   * A returned plan is internally consistent: the order is a permutation, every
//!     feedthrough node comes after the producers of its inputs.
//!   * Evaluating a returned plan is total: finite component outputs and derivatives for
//!     any input, including NaN and infinity.
//!   * Compilation is deterministic.
//!
//! The generator covers scalar signals, wide vector/wrench ports with placements, and
//! cross-frame and rigid connections that make the compiler insert transform nodes. The
//! coverage block names every error class the generator can reach; two defensive classes
//! are intentionally out of reach with the closed kind set (see the comment there).
//!
//! Deterministic by construction: the seed and the iteration count are build options, so a
//! failure is reproduced with `zig build fuzz -Dfuzz-seed=<seed> -Dfuzz-iterations=<n>`.
//! Pass `-Dfuzz-verbose=true` to print the seed, iteration count and compiled plan count.

const std = @import("std");
const base = @import("base");
const components = @import("components");
const model = @import("model");
const options = @import("fuzz_options");

const limits = base.limits;
const Rng = base.Rng;
const Dim = base.Dim;
const PortType = components.contract.PortType;
const Frame = components.contract.Frame;
const Quantity = components.contract.Quantity;
const Shape = components.contract.Shape;

const max_comps = 12;
const max_ext = 12;
const max_conn = 64;
const max_nodes = max_comps + max_conn;
const path_len = 32;

/// One external definition. A local-frame vector deliberately has no owning component and is
/// rejected only when it is used in a connection.
const ExtDef = struct {
    dim: Dim,
    shape: Shape = .scalar,
    quantity: Quantity = .scalar,
    frame: Frame = .none,
};

const ext_defs = [max_ext]ExtDef{
    .{ .dim = base.dim.length },
    .{ .dim = base.dim.speed },
    .{ .dim = base.dim.angle },
    .{ .dim = base.dim.force },
    .{ .dim = base.dim.density },
    .{ .dim = base.dim.dimensionless },
    .{ .dim = base.dim.speed, .shape = .vec3, .quantity = .velocity, .frame = .body },
    .{ .dim = base.dim.angular_rate, .shape = .vec3, .quantity = .angular_rate, .frame = .body },
    .{ .dim = base.dim.force, .shape = .vec3, .quantity = .force, .frame = .body },
    .{ .dim = .{}, .shape = .quat, .quantity = .quaternion, .frame = .body },
    .{ .dim = base.dim.force, .shape = .vec3, .quantity = .force, .frame = .world },
    .{ .dim = base.dim.force, .shape = .vec3, .quantity = .force, .frame = .local },
};

/// All storage of one generated spec. The `Spec` only borrows from it.
const Generated = struct {
    components: [max_comps]model.ComponentSpec = undefined,
    externals: [max_ext]model.ExternalSpec = undefined,
    connections: [max_conn]model.ConnectionSpec = undefined,
    comp_names: [max_comps][8]u8 = undefined,
    ext_names: [max_ext][8]u8 = undefined,
    from_text: [max_conn][path_len]u8 = undefined,
    to_text: [max_conn][path_len]u8 = undefined,
    n_comps: u32 = 0,
    n_ext: u32 = 0,
    n_conn: u32 = 0,

    fn spec(self: *const Generated) model.Spec {
        return .{
            .components = self.components[0..self.n_comps],
            .externals = self.externals[0..self.n_ext],
            .connections = self.connections[0..self.n_conn],
        };
    }
};

fn bodyModel(rng: *Rng) model.Component {
    return .{ .body = .{ .six_dof = .{
        .mass_kg = rng.range(0.1, 10.0),
        .inertia_kg_m2 = .{ .x = 1.0, .y = 2.0, .z = 2.0 },
    } } };
}

fn wingModel(rng: *Rng) model.Component {
    return .{ .wing = .{ .linear = .{
        .area_m2 = rng.range(0.1, 10.0),
        .aspect_ratio = rng.range(1.0, 12.0),
        .cl_alpha_per_rad = rng.range(3.0, 6.0),
        .cd0 = rng.range(0.0, 0.1),
        .oswald = rng.range(0.5, 1.0),
        .alpha_stall_rad = rng.range(0.1, 0.4),
    } } };
}

fn transformModel(rng: *Rng) model.Component {
    return .{ .frame_transform = .{ .apply = .{
        .mode = if (rng.below(2) == 0) .rotate else .mount,
        .dynamic = switch (rng.below(3)) {
            0 => .none,
            1 => .direct,
            else => .conjugate,
        },
    } } };
}

fn randomComponent(rng: *Rng) model.Component {
    // Parameters are mostly sane, sometimes deliberately invalid.
    const sane = rng.below(10) != 0;
    const positive = if (sane) rng.range(0.1, 10.0) else rng.range(-5.0, 0.0);
    return switch (rng.below(100)) {
        0...14 => .{ .air = .{ .isa = .{ .delta_t_k = rng.range(-20.0, 20.0) } } },
        15...29 => .{ .gain = .{ .linear = .{ .k = rng.range(-3.0, 3.0) } } },
        30...49 => .{ .lift_surface = .{ .linear = .{
            .area_m2 = positive,
            .aspect_ratio = rng.range(1.0, 12.0),
            .cl_alpha_per_rad = rng.range(3.0, 6.0),
            .cd0 = rng.range(0.0, 0.1),
            .oswald = rng.range(0.5, 1.0),
            .alpha_stall_rad = rng.range(0.1, 0.4),
        } } },
        50...64 => .{ .thruster = .{ .linear_decay = .{
            .static_thrust_n = positive,
            .max_speed_ms = rng.range(10.0, 80.0),
        } } },
        65...83 => .{ .point_mass = .{ .rigid = .{ .mass_kg = positive } } },
        84...89 => bodyModel(rng),
        90...93 => .{ .airflow = .{ .station = .{} } },
        94...96 => wingModel(rng),
        97...98 => .{ .load = .{ .point = .{} } },
        else => transformModel(rng),
    };
}

/// Mostly finite; a rare non-finite placement must produce `BadPlacement` at compile time.
fn randomPlacement(rng: *Rng) model.Placement {
    if (rng.below(60) == 0) return .{ .position_m = .{ .x = std.math.nan(f64) } };
    return .{
        .position_m = .{
            .x = rng.range(-2.0, 2.0),
            .y = rng.range(-2.0, 2.0),
            .z = rng.range(-2.0, 2.0),
        },
        .attitude_rad = .{
            .x = rng.range(-0.5, 0.5),
            .y = rng.range(-0.5, 0.5),
            .z = rng.range(-3.2, 3.2),
        },
    };
}

const SourceChoice = enum { compatible, wrong_dimension, any };
const Source = struct { text: []const u8, ptype: PortType };

fn matches(pt: PortType, want: PortType, mode: SourceChoice) bool {
    return switch (mode) {
        .compatible => components.contract.compatibility(pt, want) == null,
        .wrong_dimension => pt.shape == want.shape and pt.quantity == want.quantity and
            pt.frame == want.frame and !pt.dim.eql(want.dim),
        .any => true,
    };
}

/// A source (external or component output) for `want`. A wrong-dimension request falls back
/// to a compatible source if no candidate has that exact shape, quantity and frame.
fn pickSource(
    g: *const Generated,
    rng: *Rng,
    want: PortType,
    mode: SourceChoice,
    out: *[path_len]u8,
) ?Source {
    const cap = max_comps * limits.max_ports + max_ext;
    var texts: [cap][]const u8 = undefined;
    var types: [cap]PortType = undefined;
    var scratch: [cap][path_len]u8 = undefined;
    var n: usize = 0;

    for (g.externals[0..g.n_ext]) |e| {
        texts[n] = e.name;
        types[n] = .{
            .dim = e.dim,
            .shape = e.shape,
            .quantity = e.quantity,
            .frame = e.frame,
        };
        n += 1;
    }
    for (g.components[0..g.n_comps]) |c| {
        for (c.component.signature().outputs) |port| {
            texts[n] = std.fmt.bufPrint(&scratch[n], "{s}.{s}", .{
                c.name, port.name,
            }) catch unreachable;
            types[n] = port.ptype;
            n += 1;
        }
    }
    if (n == 0) return null;

    const chosen = choose(rng, &texts, &types, want, mode, n) orelse
        choose(rng, &texts, &types, want, .compatible, n) orelse return null;
    @memcpy(out[0..chosen.text.len], chosen.text);
    return .{ .text = out[0..chosen.text.len], .ptype = chosen.ptype };
}

fn choose(
    rng: *Rng,
    texts: []const []const u8,
    types: []const PortType,
    want: PortType,
    mode: SourceChoice,
    n: usize,
) ?Source {
    var count: usize = 0;
    for (texts[0..n], types[0..n]) |_, pt| {
        if (matches(pt, want, mode)) count += 1;
    }
    if (count == 0) return null;
    var target = rng.below(@intCast(count));
    for (texts[0..n], types[0..n]) |text, pt| {
        if (!matches(pt, want, mode)) continue;
        if (target == 0) return .{ .text = text, .ptype = pt };
        target -= 1;
    }
    unreachable;
}

const WrenchSource = struct { text: []const u8, component: u16 };

/// A component's local wrench output, used as a rigid attachment source.
fn pickWrenchSource(g: *const Generated, rng: *Rng, out: *[path_len]u8) ?WrenchSource {
    var owners: [max_comps]u16 = undefined;
    var names: [max_comps][]const u8 = undefined;
    var n: usize = 0;
    for (g.components[0..g.n_comps], 0..) |c, ci| {
        for (c.component.signature().outputs) |port| {
            if (port.ptype.shape != .wrench or port.ptype.frame != .local) continue;
            owners[n] = @intCast(ci);
            names[n] = port.name;
            n += 1;
        }
    }
    if (n == 0) return null;
    const i = rng.below(@intCast(n));
    const text = std.fmt.bufPrint(out, "{s}.{s}", .{
        g.components[owners[i]].name, names[i],
    }) catch unreachable;
    return .{ .text = text, .component = owners[i] };
}

/// A non-mount input, used as an invalid rigid attachment target.
fn pickNonMountInput(g: *const Generated, rng: *Rng, out: *[path_len]u8) ?[]const u8 {
    var texts: [max_comps * limits.max_ports][]const u8 = undefined;
    var scratch: [max_comps * limits.max_ports][path_len]u8 = undefined;
    var n: usize = 0;
    for (g.components[0..g.n_comps]) |c| {
        const sig = c.component.signature();
        for (sig.inputs) |port| {
            if (std.mem.eql(u8, port.name, "mount") and port.ptype.shape == .wrench) continue;
            texts[n] = std.fmt.bufPrint(&scratch[n], "{s}.{s}", .{
                c.name, port.name,
            }) catch unreachable;
            n += 1;
        }
    }
    if (n == 0) return null;
    const chosen = texts[rng.below(@intCast(n))];
    @memcpy(out[0..chosen.len], chosen);
    return out[0..chosen.len];
}

fn findBody(g: *const Generated) ?u16 {
    for (g.components[0..g.n_comps], 0..) |c, ci| {
        const sig = c.component.signature();
        const mi = sig.inputIndex("mount") orelse continue;
        if (sig.inputs[mi].ptype.shape == .wrench) return @intCast(ci);
    }
    return null;
}

fn addConnection(
    g: *Generated,
    from: []const u8,
    to: []const u8,
    gain: f64,
) void {
    addAttachment(g, from, to, .signal, gain);
}

fn addAttachment(
    g: *Generated,
    from: []const u8,
    to: []const u8,
    kind: model.Attachment,
    gain: f64,
) void {
    if (g.n_conn == max_conn) return;
    const i = g.n_conn;
    @memcpy(g.from_text[i][0..from.len], from);
    @memcpy(g.to_text[i][0..to.len], to);
    g.connections[i] = .{
        .from = g.from_text[i][0..from.len],
        .to = g.to_text[i][0..to.len],
        .gain = gain,
        .attachment = kind,
    };
    g.n_conn += 1;
}

fn generateExternals(g: *Generated, rng: *Rng) void {
    g.n_ext = 1 + rng.below(max_ext);
    for (0..g.n_ext) |i| {
        const name = std.fmt.bufPrint(&g.ext_names[i], "e{d}", .{i}) catch unreachable;
        const def = ext_defs[i];
        g.externals[i] = .{
            .name = name,
            .dim = def.dim,
            .shape = def.shape,
            .quantity = def.quantity,
            .frame = def.frame,
        };
    }
}

fn forceAmbiguousBodies(g: *Generated, rng: *Rng) void {
    if (g.n_comps < 2 or rng.below(30) != 0) return;
    for (0..2) |i| {
        g.components[i].component = bodyModel(rng);
        g.components[i].placement = .{};
    }
}

fn generateComponents(g: *Generated, rng: *Rng) void {
    g.n_comps = rng.below(max_comps + 1);
    for (0..g.n_comps) |i| {
        const name = std.fmt.bufPrint(&g.comp_names[i], "c{d}", .{i}) catch unreachable;
        g.components[i] = .{
            .name = name,
            .component = randomComponent(rng),
            .placement = randomPlacement(rng),
        };
    }
    forceAmbiguousBodies(g, rng);
}

fn wireInputs(g: *Generated, rng: *Rng) void {
    for (g.components[0..g.n_comps]) |c| {
        for (c.component.signature().inputs) |port| {
            const copies: u32 = switch (rng.below(20)) {
                0 => 0,
                1 => 2,
                else => 1,
            };
            for (0..copies) |_| {
                var buf: [path_len]u8 = undefined;
                var to_buf: [path_len]u8 = undefined;
                const to = std.fmt.bufPrint(&to_buf, "{s}.{s}", .{
                    c.name, port.name,
                }) catch unreachable;
                const mode: SourceChoice = if (rng.below(15) == 0)
                    .wrong_dimension
                else
                    .compatible;
                const src = pickSource(g, rng, port.ptype, mode, &buf) orelse continue;
                addConnection(g, src.text, to, if (rng.below(2) == 0) 1.0 else -1.0);
            }
        }
    }
}

/// Rigid attachments exercise the mount transform and its five error classes. The source is
/// sometimes a non-wrench signal, the target sometimes a non-mount input or a missing body.
fn addRigidAttachments(g: *Generated, rng: *Rng) void {
    if (rng.below(8) != 0) return;
    var from_buf: [path_len]u8 = undefined;
    var to_buf: [path_len]u8 = undefined;

    var source: []const u8 = undefined;
    if (rng.below(10) != 0) {
        const wrench = pickWrenchSource(g, rng, &from_buf) orelse return;
        if (rng.below(5) == 0) {
            // A bad placement on the source must be reported as `BadPlacement`.
            g.components[wrench.component].placement.position_m.y = std.math.nan(f64);
        }
        source = wrench.text;
    } else {
        const dummy: PortType = .{ .dim = base.dim.dimensionless };
        source = (pickSource(g, rng, dummy, .any, &from_buf) orelse return).text;
    }

    const body = findBody(g);
    if (body != null and rng.below(3) == 0) {
        to_buf = undefined;
        const target = pickNonMountInput(g, rng, &to_buf) orelse return;
        addAttachment(g, source, target, .rigid, 1.0);
        return;
    }
    const name = if (body) |bi| g.components[bi].name else "c0";
    const target = std.fmt.bufPrint(&to_buf, "{s}.mount", .{name}) catch unreachable;
    addAttachment(g, source, target, .rigid, 1.0);
}

fn addBadPaths(g: *Generated, rng: *Rng) void {
    if (rng.below(10) != 0) return;
    switch (rng.below(4)) {
        0 => addConnection(g, "", "c0.x", 1.0),
        1 => addConnection(g, "a.", "c0.x", 1.0),
        2 => addConnection(g, "e0", "", 1.0),
        else => addConnection(g, "nowhere", "c0.x", 1.0),
    }
}

/// A small, otherwise-valid spec with one deliberate rigid-attachment fault. The main
/// generator rarely reaches these errors because its many components often fail validation
/// first; this probe makes their coverage independent of that competition.
fn generateRigidProbe(g: *Generated, rng: *Rng) void {
    g.n_ext = 1;
    g.externals[0] = .{ .name = "e0", .dim = base.dim.dimensionless };
    g.components[0] = .{ .name = "c0", .component = bodyModel(rng), .placement = .{} };
    g.components[1] = .{ .name = "c1", .component = wingModel(rng), .placement = .{} };
    g.n_comps = 2;
    if (rng.below(2) == 0) {
        addAttachment(g, "e0", "c0.mount", .rigid, 1.0);
    } else {
        addAttachment(g, "c1.wrench", "c1.alpha", .rigid, 1.0);
    }
}

fn generate(g: *Generated, rng: *Rng) void {
    g.* = .{};
    if (rng.below(8) == 0) {
        generateRigidProbe(g, rng);
        return;
    }
    generateExternals(g, rng);
    generateComponents(g, rng);
    wireInputs(g, rng);
    addRigidAttachments(g, rng);
    addBadPaths(g, rng);
}

fn checkPlan(plan: *const model.Plan, rng: *Rng) !void {
    const allocator = std.testing.allocator;
    const n = plan.n_nodes;

    // The order is a permutation, including generated transform nodes.
    var position: [max_nodes]u32 = undefined;
    var seen: [max_nodes]bool = @splat(false);
    for (plan.order[0..n], 0..) |ni, pos| {
        try std.testing.expect(ni < n and !seen[ni]);
        seen[ni] = true;
        position[ni] = @intCast(pos);
    }

    // Every feedthrough node comes after the producers of its inputs.
    for (plan.nodes[0..n], 0..) |*node, ni| {
        const sig = node.component.signature();
        try std.testing.expectEqual(sig.inputSlots(), @as(u16, node.n_in));
        try std.testing.expectEqual(sig.outputSlots(), @as(u16, node.n_out));
        try std.testing.expectEqual(sig.stateSlots(), @as(u16, node.n_state));
        for (sig.inputs, 0..) |port, pi| {
            try std.testing.expectEqual(port.ptype.width(), node.inputs[pi].width);
        }
        if (!sig.feedthrough) continue;
        for (node.inputs[0..sig.inputs.len]) |wiring| {
            for (plan.links[wiring.first..][0..wiring.count]) |link| {
                if (link.source < plan.n_external_slots) continue;
                const producer = producerOf(plan, link.source);
                try std.testing.expect(position[producer] < position[ni]);
            }
        }
    }

    // Evaluation is total. Exact storage: the same contract an embedder gets.
    const ws_bytes = model.Workspace.bytesNeeded(plan);
    const ws_storage = try allocator.alignedAlloc(
        u8,
        .fromByteUnits(model.storage_alignment),
        ws_bytes,
    );
    defer allocator.free(ws_storage);
    const ws = model.Workspace.into(plan, ws_storage);

    const x = try allocator.alloc(f64, plan.n_states);
    defer allocator.free(x);
    const dx = try allocator.alloc(f64, plan.n_states);
    defer allocator.free(dx);
    const ext = try allocator.alloc(f64, plan.n_external_slots);
    defer allocator.free(ext);

    const inf = std.math.inf(f64);
    const specials = [_]f64{ std.math.nan(f64), inf, -inf, 1.0e308, 0.0 };
    for (0..4) |round| {
        for (x) |*v| {
            v.* = if (round == 0) rng.range(-50.0, 50.0) else pick(&specials, rng, round);
        }
        for (ext) |*v| {
            v.* = if (round < 2) rng.range(-50.0, 50.0) else pick(&specials, rng, round);
        }
        _ = plan.rhs(x, ext, ws, dx);
        for (ws.signals[plan.n_external_slots..plan.n_signals]) |v| {
            try std.testing.expect(std.math.isFinite(v));
        }
        for (dx) |v| try std.testing.expect(std.math.isFinite(v));
    }
}

fn pick(values: []const f64, rng: *Rng, round: usize) f64 {
    // Mix special values with ordinary ones so that not every input is poisoned.
    if (rng.below(3) == 0) return rng.range(-10.0, 10.0);
    return values[(rng.below(@intCast(values.len)) + round) % values.len];
}

fn producerOf(plan: *const model.Plan, signal: u16) u32 {
    const offset = signal - plan.n_external_slots;
    for (plan.nodes[0..plan.n_nodes], 0..) |*node, ni| {
        const end = node.out_offset + @as(u16, node.n_out);
        if (offset >= node.out_offset and offset < end) return @intCast(ni);
    }
    unreachable;
}

/// Tally of compile errors by name (error sets are not enums, so key by `@errorName`).
const ErrorCounts = struct {
    names: [32][]const u8 = undefined,
    counts: [32]u32 = @splat(0),
    len: usize = 0,

    fn add(self: *ErrorCounts, err: anyerror) void {
        const name = @errorName(err);
        for (self.names[0..self.len], 0..) |known, i| {
            if (std.mem.eql(u8, known, name)) {
                self.counts[i] += 1;
                return;
            }
        }
        self.names[self.len] = name;
        self.counts[self.len] = 1;
        self.len += 1;
    }

    fn count(self: *const ErrorCounts, err: anyerror) u32 {
        const name = @errorName(err);
        for (self.names[0..self.len], 0..) |known, i| {
            if (std.mem.eql(u8, known, name)) return self.counts[i];
        }
        return 0;
    }
};

fn printSummary(compiled: u32, seen: *const ErrorCounts) void {
    std.debug.print(
        "fuzz: seed=0x{x} iterations={d} compiled={d}\n",
        .{ options.seed, options.iterations, compiled },
    );
    for (seen.names[0..seen.len], seen.counts[0..seen.len]) |name, count| {
        std.debug.print("  {s}: {d}\n", .{ name, count });
    }
}

test "random architectures: compile is total and plans are sound" {
    const allocator = std.testing.allocator;
    var rng = Rng.init(options.seed);
    var gen: Generated = undefined;

    var compiled: u32 = 0;
    var seen_errors: ErrorCounts = .{};

    for (0..options.iterations) |_| {
        generate(&gen, &rng);
        const spec = gen.spec();

        var diag: model.Diagnostic = .{};
        const storage_a = try allocator.alignedAlloc(
            u8,
            .fromByteUnits(model.storage_alignment),
            model.bytesNeeded(&spec),
        );
        defer allocator.free(storage_a);

        if (model.compileInto(storage_a, &spec, &diag)) |plan_a| {
            compiled += 1;
            try checkPlan(plan_a, &rng);

            // Determinism: the same spec compiles to the same order and layout.
            var diag_b: model.Diagnostic = .{};
            const storage_b = try allocator.alignedAlloc(
                u8,
                .fromByteUnits(model.storage_alignment),
                model.bytesNeeded(&spec),
            );
            defer allocator.free(storage_b);
            const plan_b = try model.compileInto(storage_b, &spec, &diag_b);
            try std.testing.expectEqual(plan_a.n_nodes, plan_b.n_nodes);
            const order_a = plan_a.order[0..plan_a.n_nodes];
            const order_b = plan_b.order[0..plan_b.n_nodes];
            try std.testing.expectEqualSlices(u16, order_a, order_b);
            try std.testing.expectEqual(plan_a.n_links, plan_b.n_links);
        } else |err| {
            seen_errors.add(err);
            if (diag.message().len == 0) {
                std.debug.print("error {s} without a diagnostic\n", .{@errorName(err)});
                return error.MissingDiagnostic;
            }
        }
    }

    // The generator must reach both sides: plans worth checking and the errors the compiler
    // is meant to catch. `BodyHasNoAttitude` and `TooManyGeneratedNodes` are defensive paths:
    // the closed kind set has no body without an attitude output, and 64 connections cannot
    // exceed the u16 generated-node ceiling. They are not fuzz-reachable by construction.
    if (options.iterations >= 1000) {
        try std.testing.expect(compiled > options.iterations / 20);
        for ([_]model.CompileError{
            error.DimensionMismatch, error.UnconnectedInput,         error.MultiplyDriven,
            error.AlgebraicLoop,     error.InvalidParameter,         error.UnknownExternal,
            error.BadPlacement,      error.BadPath,                  error.FrameMismatch,
            error.AmbiguousBody,     error.RigidAttachmentNotWrench, error.RigidAttachmentToNonBody,
        }) |expected| {
            if (seen_errors.count(expected) == 0) {
                std.debug.print("fuzzer never produced {s}\n", .{@errorName(expected)});
                return error.CoverageGap;
            }
        }
    }
    if (options.verbose) printSummary(compiled, &seen_errors);
}
