//! The component contract: what every physical building block must look like.
//!
//! Vocabulary
//!   kind       A physical role with a fixed signature (a lift surface, a thruster, ...).
//!              Implemented as a tagged union called `Model`.
//!   model      One interchangeable implementation of a kind at some fidelity (an analytic
//!              lifting-line wing, a table of measured polars, ...). All models of a kind
//!              share the kind's signature, so swapping one for another never changes how
//!              the vehicle is wired.
//!   signature  Inputs, outputs, continuous states and their physical dimensions.
//!
//! A kind is a `union(enum)` declaring:
//!   In, Out, State        structs of f64: one field per port
//!   signature             built by `signature()` from the three structs above
//!   validate(self)        ?[]const u8: null when the parameters are usable
//!   outputs(self, x, u)   y = g(x, u)          -> Evaluated(Out)
//!   derivatives(self,x,u) dx/dt = f(x, u)      -> Evaluated(State), stateful kinds only
//!
//! Dynamics are built in from the start. A static component has no states and its outputs
//! depend on its inputs (`feedthrough`). A dynamic component has states; if its outputs
//! depend on states only (`feedthrough = false`), it breaks algebraic loops, which is what
//! lets "velocity -> drag -> force -> acceleration -> velocity" be wired without a solver.
//!
//! Error policy
//!   * Bad parameters supplied by a person are an operating error, caught by `validate`
//!     when the model is compiled. Inside `outputs` they are asserted as invariants.
//!   * Inputs outside the range of validity are not errors: they come back as a finite
//!     result flagged `beyond_envelope` or `invalid`.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");

const Dim = base.Dim;
const Validity = base.Validity;
const limits = base.limits;

pub const FanIn = enum {
    /// Exactly one source must drive the port.
    single,
    /// Any number of sources (including none) are added up. Used for forces and masses.
    sum,
};

pub const Port = struct {
    name: []const u8,
    dim: Dim,
    fan_in: FanIn = .single,
};

pub const Signature = struct {
    inputs: []const Port,
    outputs: []const Port,
    /// Dimension of each continuous state. Its time derivative has this dimension / s.
    states: []const Port,
    /// True when some output algebraically depends on some input. Must be true for
    /// stateless kinds. A stateful kind with false breaks algebraic loops.
    feedthrough: bool,

    pub fn inputIndex(self: Signature, name: []const u8) ?u32 {
        return indexOf(self.inputs, name);
    }

    pub fn outputIndex(self: Signature, name: []const u8) ?u32 {
        return indexOf(self.outputs, name);
    }

    pub fn stateIndex(self: Signature, name: []const u8) ?u32 {
        return indexOf(self.states, name);
    }
};

fn indexOf(ports: []const Port, name: []const u8) ?u32 {
    for (ports, 0..) |p, i| {
        if (std.mem.eql(u8, p.name, name)) return @intCast(i);
    }
    return null;
}

/// Result of an evaluation: the value and how far it can be trusted.
pub fn Evaluated(comptime T: type) type {
    return struct {
        value: T,
        validity: Validity = .valid,
    };
}

/// null when the parameters are usable, otherwise a message naming the offending one.
pub const ParamCheck = ?[]const u8;

pub fn isPositive(x: f64) bool {
    return x > 0.0 and std.math.isFinite(x);
}

pub fn isNonNegative(x: f64) bool {
    return x >= 0.0 and std.math.isFinite(x);
}

pub const isFinite = std.math.isFinite;

/// Asserts that every field of a port struct is finite. Models call this on their inputs.
pub fn assertFiniteFields(value: anytype) void {
    inline for (@typeInfo(@TypeOf(value)).@"struct".field_names) |name| {
        assert(std.math.isFinite(@field(value, name)));
    }
}

/// Builds the signature of kind `M` from its In/Out/State structs and a dimension for each
/// port. Everything is checked at compile time: a missing or superfluous dimension, a field
/// that is not f64, or a sum port that does not exist is a compile error.
///
///   .in_dims .out_dims .state_dims   `.{ .port_name = dim.speed, ... }`
///   .in_sum                          `.{ .port_name = true }` marks summing inputs
///   .feedthrough                     bool
pub fn signature(comptime M: type, comptime opts: anytype) Signature {
    comptime {
        const Opts = @TypeOf(opts);
        const in_sum = if (@hasField(Opts, "in_sum")) opts.in_sum else .{};
        const state_dims = if (@hasField(Opts, "state_dims")) opts.state_dims else .{};
        return .{
            .inputs = portsFrom(M.In, opts.in_dims, in_sum),
            .outputs = portsFrom(M.Out, opts.out_dims, .{}),
            .states = portsFrom(M.State, state_dims, .{}),
            .feedthrough = opts.feedthrough,
        };
    }
}

fn portsFrom(comptime T: type, comptime dims: anytype, comptime sums: anytype) []const Port {
    comptime {
        const info = @typeInfo(T).@"struct";
        for (@typeInfo(@TypeOf(dims)).@"struct".field_names) |name| {
            if (!@hasField(T, name)) {
                @compileError("dimension given for unknown port '" ++ name ++ "' of " ++
                    @typeName(T));
            }
        }
        for (@typeInfo(@TypeOf(sums)).@"struct".field_names) |name| {
            if (!@hasField(T, name)) {
                @compileError("summing flag given for unknown port '" ++ name ++ "' of " ++
                    @typeName(T));
            }
        }
        var ports: [info.field_names.len]Port = undefined;
        for (info.field_names, info.field_types, 0..) |name, field_type, i| {
            if (field_type != f64) {
                @compileError("port '" ++ name ++ "' of " ++ @typeName(T) ++ " must be f64");
            }
            if (!@hasField(@TypeOf(dims), name)) {
                @compileError("no dimension given for port '" ++ name ++ "' of " ++
                    @typeName(T));
            }
            ports[i] = .{
                .name = name,
                .dim = @field(dims, name),
                .fan_in = if (@hasField(@TypeOf(sums), name)) .sum else .single,
            };
        }
        const final = ports;
        return &final;
    }
}

/// Compile-time proof that `M` honors the contract. The registry runs it on every kind.
pub fn assertKind(comptime M: type) void {
    comptime {
        switch (@typeInfo(M)) {
            .@"union" => {},
            else => @compileError(@typeName(M) ++ ": a kind must be a union(enum) of models"),
        }
        const required = [_][]const u8{ "In", "Out", "State", "signature", "validate", "outputs" };
        for (required) |decl| {
            if (!@hasDecl(M, decl)) {
                @compileError(@typeName(M) ++ " is missing required declaration '" ++ decl ++ "'");
            }
        }
        const sig: Signature = M.signature;
        const too_many = sig.inputs.len > limits.max_ports or
            sig.outputs.len > limits.max_ports or sig.states.len > limits.max_ports;
        if (too_many) @compileError(@typeName(M) ++ " exceeds limits.max_ports");
        const stateful = sig.states.len > 0;
        if (stateful != @hasDecl(M, "derivatives")) {
            @compileError(@typeName(M) ++ ": `derivatives` must exist exactly when it has states");
        }
        if (!stateful and !sig.feedthrough) {
            @compileError(@typeName(M) ++ ": a stateless kind must have feedthrough = true");
        }
    }
}

pub fn fromSlice(comptime T: type, values: []const f64) T {
    const names = @typeInfo(T).@"struct".field_names;
    assert(values.len == names.len);
    if (comptime names.len == 0) return .{};
    var result: T = undefined;
    inline for (names, 0..) |name, i| @field(result, name) = values[i];
    return result;
}

pub fn toSlice(comptime T: type, value: T, out: []f64) void {
    const names = @typeInfo(T).@"struct".field_names;
    assert(out.len == names.len);
    inline for (names, 0..) |name, i| out[i] = @field(value, name);
}

/// Flat-slice form of `outputs`, used by the compiled plan.
pub fn outputsFlat(
    comptime M: type,
    model: *const M,
    x: []const f64,
    u: []const f64,
    y: []f64,
) Validity {
    const r = model.outputs(fromSlice(M.State, x), fromSlice(M.In, u));
    toSlice(M.Out, r.value, y);
    return r.validity;
}

/// Flat-slice form of `derivatives`. Stateless kinds have nothing to compute.
pub fn derivativesFlat(
    comptime M: type,
    model: *const M,
    x: []const f64,
    u: []const f64,
    dx: []f64,
) Validity {
    if (comptime M.signature.states.len == 0) {
        assert(dx.len == 0);
        return .valid;
    } else {
        const r = model.derivatives(fromSlice(M.State, x), fromSlice(M.In, u));
        toSlice(M.State, r.value, dx);
        return r.validity;
    }
}

// ---------------------------------------------------------------------------------------
// Reusable property test: every model of every kind must pass this.
// ---------------------------------------------------------------------------------------

fn randomFill(values: []f64, rng: *base.Rng) void {
    for (values) |*v| {
        v.* = switch (rng.below(8)) {
            0 => 0.0,
            1 => rng.range(-1.0e6, 1.0e6),
            else => rng.range(-100.0, 100.0),
        };
    }
}

fn sameBits(a: []const f64, b: []const f64) bool {
    return std.mem.eql(u8, std.mem.sliceAsBytes(a), std.mem.sliceAsBytes(b));
}

/// Checks, over `samples` random points that deliberately include out-of-range values:
///   * outputs and derivatives are finite for any finite input (the optimizer may probe
///     anywhere, so a model must degrade to a flagged result, never to NaN);
///   * evaluation is deterministic, bit for bit;
///   * for kinds declared `feedthrough = false`, outputs ignore the inputs entirely
///     (verified by replacing the inputs with NaN).
pub fn expectKindContract(
    comptime M: type,
    model: *const M,
    rng: *base.Rng,
    samples: u32,
) !void {
    const sig = M.signature;
    const ni = sig.inputs.len;
    const no = sig.outputs.len;
    const ns = sig.states.len;

    var x: [limits.max_ports]f64 = undefined;
    var u: [limits.max_ports]f64 = undefined;
    var y1: [limits.max_ports]f64 = undefined;
    var y2: [limits.max_ports]f64 = undefined;
    var dx1: [limits.max_ports]f64 = undefined;
    var dx2: [limits.max_ports]f64 = undefined;

    for (0..samples) |_| {
        randomFill(x[0..ns], rng);
        randomFill(u[0..ni], rng);

        const v1 = outputsFlat(M, model, x[0..ns], u[0..ni], y1[0..no]);
        const v2 = outputsFlat(M, model, x[0..ns], u[0..ni], y2[0..no]);
        for (y1[0..no]) |value| try std.testing.expect(std.math.isFinite(value));
        try std.testing.expectEqual(v1, v2);
        try std.testing.expect(sameBits(y1[0..no], y2[0..no]));

        if (!sig.feedthrough) {
            @memset(u[0..ni], std.math.nan(f64));
            _ = outputsFlat(M, model, x[0..ns], u[0..ni], y2[0..no]);
            try std.testing.expect(sameBits(y1[0..no], y2[0..no]));
            randomFill(u[0..ni], rng);
        }

        if (ns > 0) {
            const d1 = derivativesFlat(M, model, x[0..ns], u[0..ni], dx1[0..ns]);
            const d2 = derivativesFlat(M, model, x[0..ns], u[0..ni], dx2[0..ns]);
            for (dx1[0..ns]) |value| try std.testing.expect(std.math.isFinite(value));
            try std.testing.expectEqual(d1, d2);
            try std.testing.expect(sameBits(dx1[0..ns], dx2[0..ns]));
        }
    }
}

// ---------------------------------------------------------------------------------------
// Tests of the contract machinery itself, on a toy kind.
// ---------------------------------------------------------------------------------------

const toy = struct {
    const Model = union(enum) {
        spring: Spring,

        pub const In = struct { force: f64, offset: f64 };
        pub const Out = struct { position: f64 };
        pub const State = struct { position: f64, velocity: f64 };
        pub const signature = contract_signature;

        pub fn validate(self: *const Model) ParamCheck {
            return switch (self.*) {
                inline else => |*m| if (isPositive(m.k)) null else "k must be positive",
            };
        }

        pub fn outputs(self: *const Model, x: State, u: In) Evaluated(Out) {
            _ = self;
            _ = u;
            return .{ .value = .{ .position = x.position } };
        }

        pub fn derivatives(self: *const Model, x: State, u: In) Evaluated(State) {
            return switch (self.*) {
                inline else => |*m| .{ .value = .{
                    .position = x.velocity,
                    .velocity = (u.force - m.k * (x.position - u.offset)) / m.mass,
                } },
            };
        }
    };

    const Spring = struct { k: f64, mass: f64 };

    const contract_signature = signature(Model, .{
        .in_dims = .{ .force = base.dim.force, .offset = base.dim.length },
        .in_sum = .{ .force = true },
        .out_dims = .{ .position = base.dim.length },
        .state_dims = .{ .position = base.dim.length, .velocity = base.dim.speed },
        .feedthrough = false,
    });
};

test "signature is built from the port structs" {
    const sig = toy.Model.signature;
    try std.testing.expectEqual(@as(usize, 2), sig.inputs.len);
    try std.testing.expectEqualStrings("force", sig.inputs[0].name);
    try std.testing.expectEqual(FanIn.sum, sig.inputs[0].fan_in);
    try std.testing.expectEqual(FanIn.single, sig.inputs[1].fan_in);
    try std.testing.expect(sig.inputs[0].dim.eql(base.dim.force));
    try std.testing.expectEqual(@as(?u32, 1), sig.inputIndex("offset"));
    try std.testing.expectEqual(@as(?u32, null), sig.inputIndex("nope"));
    try std.testing.expectEqual(@as(?u32, 1), sig.stateIndex("velocity"));
    try std.testing.expect(!sig.feedthrough);
    assertKind(toy.Model);
}

test "flat marshalling round-trips and matches the typed call" {
    const m: toy.Model = .{ .spring = .{ .k = 4.0, .mass = 2.0 } };
    const x = [_]f64{ 0.5, -1.0 };
    const u = [_]f64{ 10.0, 0.25 };
    var dx: [2]f64 = undefined;
    _ = derivativesFlat(toy.Model, &m, &x, &u, &dx);
    const typed = m.derivatives(fromSlice(toy.Model.State, &x), fromSlice(toy.Model.In, &u));
    try std.testing.expectEqual(typed.value.position, dx[0]);
    try std.testing.expectEqual(typed.value.velocity, dx[1]);
    // (10 - 4 * (0.5 - 0.25)) / 2 = 4.5
    try std.testing.expectApproxEqAbs(@as(f64, 4.5), dx[1], 1e-15);
}

test "the generic property test accepts a well-behaved kind" {
    const m: toy.Model = .{ .spring = .{ .k = 4.0, .mass = 2.0 } };
    var rng = base.Rng.init(1);
    try expectKindContract(toy.Model, &m, &rng, 500);
}

test "validity helpers" {
    try std.testing.expect(isPositive(1e-300));
    try std.testing.expect(!isPositive(0.0));
    try std.testing.expect(!isPositive(-1.0));
    try std.testing.expect(!isPositive(std.math.nan(f64)));
    try std.testing.expect(!isPositive(std.math.inf(f64)));
    try std.testing.expect(isNonNegative(0.0));
    try std.testing.expect(!isNonNegative(-1e-9));
}
