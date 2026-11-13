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
//! Deterministic by construction: the seed and the iteration count are build options, so a
//! failure is reproduced with `zig build fuzz -Dfuzz-seed=<seed> -Dfuzz-iterations=<n>`.
//! Pass `-Dfuzz-verbose=true` to print the seed, iteration count and compiled plan count.

const std = @import("std");
const base = @import("base");
const model = @import("model");
const options = @import("fuzz_options");

const limits = base.limits;
const Rng = base.Rng;
const Dim = base.Dim;

const max_comps = 12;
const max_ext = 6;
const max_conn = 64;

const ext_dims = [max_ext]Dim{
    base.dim.length,
    base.dim.speed,
    base.dim.angle,
    base.dim.force,
    base.dim.density,
    base.dim.dimensionless,
};

/// All storage of one generated spec. The `Spec` only borrows from it.
const Generated = struct {
    components: [max_comps]model.ComponentSpec = undefined,
    externals: [max_ext]model.ExternalSpec = undefined,
    connections: [max_conn]model.ConnectionSpec = undefined,
    comp_names: [max_comps][8]u8 = undefined,
    ext_names: [max_ext][8]u8 = undefined,
    from_text: [max_conn][24]u8 = undefined,
    to_text: [max_conn][24]u8 = undefined,
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

fn randomComponent(rng: *Rng) model.Component {
    // Parameters are mostly sane, sometimes deliberately invalid.
    const sane = rng.below(10) != 0;
    const positive = if (sane) rng.range(0.1, 10.0) else rng.range(-5.0, 0.0);
    return switch (rng.below(5)) {
        0 => .{ .air = .{ .isa = .{ .delta_t_k = rng.range(-20.0, 20.0) } } },
        1 => .{ .gain = .{ .linear = .{ .k = rng.range(-3.0, 3.0) } } },
        2 => .{ .lift_surface = .{ .linear = .{
            .area_m2 = positive,
            .aspect_ratio = rng.range(1.0, 12.0),
            .cl_alpha_per_rad = rng.range(3.0, 6.0),
            .cd0 = rng.range(0.0, 0.1),
            .oswald = rng.range(0.5, 1.0),
            .alpha_stall_rad = rng.range(0.1, 0.4),
        } } },
        3 => .{ .thruster = .{ .linear_decay = .{
            .static_thrust_n = positive,
            .max_speed_ms = rng.range(10.0, 80.0),
        } } },
        else => .{ .point_mass = .{ .rigid = .{ .mass_kg = positive } } },
    };
}

/// A source (external or component output) of the given dimension, or null if none exists.
fn pickSource(g: *Generated, rng: *Rng, dim: Dim, out: *[24]u8) ?[]const u8 {
    var candidates: [max_comps * limits.max_ports + max_ext][]const u8 = undefined;
    var n: usize = 0;
    var scratch: [max_comps * limits.max_ports + max_ext][24]u8 = undefined;

    for (g.externals[0..g.n_ext]) |e| {
        if (e.dim.eql(dim)) {
            candidates[n] = e.name;
            n += 1;
        }
    }
    for (g.components[0..g.n_comps]) |c| {
        for (c.component.signature().outputs) |port| {
            if (!port.ptype.dim.eql(dim)) continue;
            const text = std.fmt.bufPrint(&scratch[n], "{s}.{s}", .{
                c.name, port.name,
            }) catch unreachable;
            candidates[n] = text;
            n += 1;
        }
    }
    if (n == 0) return null;
    const chosen = candidates[rng.below(@intCast(n))];
    @memcpy(out[0..chosen.len], chosen);
    return out[0..chosen.len];
}

fn addConnection(g: *Generated, from: []const u8, to: []const u8, gain: f64) void {
    if (g.n_conn == max_conn) return;
    const i = g.n_conn;
    @memcpy(g.from_text[i][0..from.len], from);
    @memcpy(g.to_text[i][0..to.len], to);
    g.connections[i] = .{
        .from = g.from_text[i][0..from.len],
        .to = g.to_text[i][0..to.len],
        .gain = gain,
    };
    g.n_conn += 1;
}

fn generate(g: *Generated, rng: *Rng) void {
    g.* = .{};
    g.n_ext = 1 + rng.below(max_ext);
    for (0..g.n_ext) |i| {
        const name = std.fmt.bufPrint(&g.ext_names[i], "e{d}", .{i}) catch unreachable;
        g.externals[i] = .{ .name = name, .dim = ext_dims[i] };
    }
    g.n_comps = rng.below(max_comps + 1);
    for (0..g.n_comps) |i| {
        const name = std.fmt.bufPrint(&g.comp_names[i], "c{d}", .{i}) catch unreachable;
        g.components[i] = .{ .name = name, .component = randomComponent(rng) };
    }

    // Wire the inputs of every component: mostly compatible, sometimes missing, doubled or
    // nonsensical, so that every error class is exercised.
    for (g.components[0..g.n_comps]) |c| {
        for (c.component.signature().inputs) |port| {
            const copies: u32 = switch (rng.below(20)) {
                0 => 0,
                1 => 2,
                else => 1,
            };
            for (0..copies) |_| {
                var buf: [24]u8 = undefined;
                var to_buf: [24]u8 = undefined;
                const to = std.fmt.bufPrint(&to_buf, "{s}.{s}", .{
                    c.name, port.name,
                }) catch unreachable;
                // Occasionally wire a source of a different (but existing) dimension.
                const dim = if (rng.below(15) == 0)
                    ext_dims[rng.below(max_ext)]
                else
                    port.ptype.dim;
                const from = pickSource(g, rng, dim, &buf) orelse continue;
                const gain: f64 = if (rng.below(2) == 0) 1.0 else -1.0;
                addConnection(g, from, to, gain);
            }
        }
    }
    if (rng.below(25) == 0) addConnection(g, "nowhere", "c0.x", 1.0);
}

fn checkPlan(plan: *const model.Plan, rng: *Rng) !void {
    const allocator = std.testing.allocator;
    const n = plan.n_nodes;

    // The order is a permutation.
    var position: [max_comps]u32 = undefined;
    var seen: [max_comps]bool = @splat(false);
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
                if (link.source < plan.n_externals) continue;
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
    const ext = try allocator.alloc(f64, plan.n_externals);
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
        for (ws.signals[plan.n_externals..plan.n_signals]) |v| {
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
    const offset = signal - plan.n_externals;
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

    // The generator must actually reach both sides: plans worth checking, and the errors
    // the compiler is meant to catch.
    if (options.iterations >= 1000) {
        try std.testing.expect(compiled > options.iterations / 20);
        for ([_]model.CompileError{
            error.DimensionMismatch, error.UnconnectedInput, error.MultiplyDriven,
            error.AlgebraicLoop,     error.InvalidParameter, error.UnknownExternal,
        }) |expected| {
            if (seen_errors.count(expected) == 0) {
                std.debug.print("fuzzer never produced {s}\n", .{@errorName(expected)});
                return error.CoverageGap;
            }
        }
    }
    if (options.verbose) {
        std.debug.print(
            "fuzz: seed=0x{x} iterations={d} compiled={d}\n",
            .{ options.seed, options.iterations, compiled },
        );
    }
}
