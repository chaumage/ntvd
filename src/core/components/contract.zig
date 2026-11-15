//! The component contract: what every physical building block must look like.
//!
//! Vocabulary
//!   kind       A physical role with a fixed signature (a lift surface, a thruster, ...).
//!              Implemented as a tagged union called `Model`.
//!   model      One interchangeable implementation of a kind at some fidelity (an analytic
//!              lifting-line wing, a table of measured polars, ...). All models of a kind
//!              share the kind's signature, so swapping one for another never changes how
//!              the vehicle is wired.
//!   signature  Inputs, outputs, continuous states and the type of each port.
//!
//! A kind is a `union(enum)` declaring:
//!   In, Out, State        structs of f64, Vec3, Quat or Wrench: one field per port
//!   signature             built by `signature()` from the three structs above
//!   validate(self)        ?[]const u8: null when the parameters are usable
//!   outputs(self, x, u)   y = g(x, u)          -> Evaluated(Out)
//!   derivatives(self,x,u) dx/dt = f(x, u)      -> Evaluated(State), stateful kinds only
//!
//! A port has a shape (scalar, vec3, quat or wrench), a physical dimension, a quantity and
//! a frame. The shape is derived from the field type; the rest is authored with helpers such
//! as `scalar(dim.speed)` or `vec3(dim.force, .force, .body)`. Wide ports occupy several
//! consecutive slots in the flat signal and state arrays.
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

/// How a port is laid out in the flat signal and state arrays.
pub const Shape = enum { scalar, vec3, quat, wrench };

/// What a port physically carries. The quantity fixes the dimension of a vector port and
/// tells the compiler whether two ports may be connected.
pub const Quantity = enum {
    scalar, // no frame
    position, // m
    velocity, // m/s
    acceleration, // m/s^2
    force, // N
    moment, // N*m
    wrench, // force + moment at one point
    angular_rate, // rad/s
    quaternion, // orientation
};

/// Frame a vector port is expressed in. Scalars never have a frame.
pub const Frame = enum { none, world, body, local };

pub const PortType = struct {
    dim: Dim,
    shape: Shape = .scalar,
    quantity: Quantity = .scalar,
    frame: Frame = .none,
    fan_in: FanIn = .single,

    /// Flat slots occupied in the signal and state arrays.
    pub fn width(self: PortType) u8 {
        return switch (self.shape) {
            .scalar => 1,
            .vec3 => 3,
            .quat => 4,
            .wrench => 6, // force xyz, then moment xyz
        };
    }
};

pub const Port = struct {
    name: []const u8,
    ptype: PortType,
};

/// Per-component information supplied by the compiled plan, never stored in the model.
/// A kind that needs its own placement reads it here; kinds that do not use it ignore it.
pub const Context = struct {
    placement: base.Placement = .{},
};

/// A scalar port: one slot, no frame.
pub fn scalar(dim: Dim) PortType {
    return .{ .dim = dim };
}

/// A 3-vector port in `frame`.
pub fn vec3(dim: Dim, quantity: Quantity, frame: Frame) PortType {
    return .{ .dim = dim, .shape = .vec3, .quantity = quantity, .frame = frame };
}

/// A unit-quaternion orientation port in `frame`.
pub fn quat(frame: Frame) PortType {
    return .{ .dim = .{}, .shape = .quat, .quantity = .quaternion, .frame = frame };
}

/// A force-and-moment port in `frame`; the two parts carry their own dimensions.
pub fn wrench(frame: Frame) PortType {
    return .{ .dim = .{}, .shape = .wrench, .quantity = .wrench, .frame = frame };
}

pub const Signature = struct {
    inputs: []const Port,
    outputs: []const Port,
    /// Continuous states. The time derivative of a state has the same slots.
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

    /// Slots occupied by all input ports of one component.
    pub fn inputSlots(self: Signature) u16 {
        return slots(self.inputs);
    }

    pub fn outputSlots(self: Signature) u16 {
        return slots(self.outputs);
    }

    pub fn stateSlots(self: Signature) u16 {
        return slots(self.states);
    }
};

fn indexOf(ports: []const Port, name: []const u8) ?u32 {
    for (ports, 0..) |p, i| {
        if (std.mem.eql(u8, p.name, name)) return @intCast(i);
    }
    return null;
}

fn slots(ports: []const Port) u16 {
    var n: u16 = 0;
    for (ports) |p| n += @as(u16, p.ptype.width());
    return n;
}

/// Slots occupied by the ports before `index`, i.e. the slot offset of that port.
pub fn slotsBefore(ports: []const Port, index: u32) u16 {
    var n: u16 = 0;
    for (ports[0..index]) |p| n += @as(u16, p.ptype.width());
    return n;
}

/// Why two port types cannot be connected, or null when they can.
pub const Incompatibility = enum { shape, quantity, frame, dimension };

pub fn compatibility(from: PortType, to: PortType) ?Incompatibility {
    if (from.shape != to.shape) return .shape;
    if (from.quantity != to.quantity) return .quantity;
    if (from.frame != to.frame) return .frame;
    if (from.shape != .wrench and !from.dim.eql(to.dim)) return .dimension;
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

/// Asserts that every scalar field of a port struct, recursively, is finite.
/// Models call this on their inputs.
pub fn assertFiniteFields(value: anytype) void {
    const T = @TypeOf(value);
    switch (@typeInfo(T)) {
        .float => assert(std.math.isFinite(value)),
        .@"struct" => |info| inline for (info.field_names) |name| {
            assertFiniteFields(@field(value, name));
        },
        else => @compileError("port field must be f64, Vec3, Quat or Wrench, not " ++
            @typeName(T)),
    }
}

/// Number of flat slots a port struct occupies.
pub fn slotCount(comptime T: type) u16 {
    const info = @typeInfo(T).@"struct";
    var n: u16 = 0;
    inline for (info.field_types) |FT| n += widthOf(FT);
    return n;
}

/// Builds the signature of kind `M` from its In/Out/State structs and one port type per
/// field. Everything is checked at compile time: a missing or superfluous entry, a field
/// that is not f64/Vec3/Quat/Wrench, or a port whose shape does not match its field type is
/// a compile error.
///
///   .in .out .state   `.{ .port_name = scalar(dim.speed), ... }`
///   .in_sum           `.{ .port_name = true }` marks summing inputs
///   .feedthrough      bool
pub fn signature(comptime M: type, comptime opts: anytype) Signature {
    comptime {
        const Opts = @TypeOf(opts);
        const in_sum = if (@hasField(Opts, "in_sum")) opts.in_sum else .{};
        const state_types = if (@hasField(Opts, "state")) opts.state else .{};
        return .{
            .inputs = portsFrom(M.In, opts.in, in_sum, true),
            .outputs = portsFrom(M.Out, opts.out, .{}, false),
            .states = portsFrom(M.State, state_types, .{}, false),
            .feedthrough = opts.feedthrough,
        };
    }
}

fn portsFrom(
    comptime T: type,
    comptime types: anytype,
    comptime sums: anytype,
    comptime is_input: bool,
) []const Port {
    comptime {
        const info = @typeInfo(T).@"struct";
        const TypeTable = @TypeOf(types);
        for (@typeInfo(TypeTable).@"struct".field_names) |name| {
            if (!@hasField(T, name)) {
                @compileError("port type given for unknown port '" ++ name ++ "' of " ++
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
            const shape = shapeOf(name, field_type);
            if (!@hasField(TypeTable, name)) {
                @compileError("no port type given for port '" ++ name ++ "' of " ++
                    @typeName(T));
            }
            var ptype: PortType = @field(types, name);
            if (ptype.shape != shape) {
                @compileError("port '" ++ name ++ "' of " ++ @typeName(T) ++ " is " ++
                    @tagName(shape) ++ " but declares shape " ++ @tagName(ptype.shape));
            }
            if (!is_input and ptype.fan_in != .single) {
                @compileError("port '" ++ name ++ "' of " ++ @typeName(T) ++
                    " is not an input and cannot sum");
            }
            if (is_input and @hasField(@TypeOf(sums), name)) ptype.fan_in = .sum;
            checkPortType(name, T, ptype);
            ports[i] = .{ .name = name, .ptype = ptype };
        }
        const final = ports;
        return &final;
    }
}

fn shapeOf(comptime name: []const u8, comptime T: type) Shape {
    if (T == f64) return .scalar;
    if (T == base.Vec3) return .vec3;
    if (T == base.Quat) return .quat;
    if (T == base.Wrench) return .wrench;
    @compileError("port '" ++ name ++ "' must be f64, Vec3, Quat or Wrench, not " ++
        @typeName(T));
}

fn widthOf(comptime T: type) u8 {
    if (T == f64) return 1;
    if (T == base.Vec3) return 3;
    if (T == base.Quat) return 4;
    if (T == base.Wrench) return 6;
    @compileError("port field must be f64, Vec3, Quat or Wrench, not " ++ @typeName(T));
}

/// Dimension implied by a physical vector quantity, or null when the quantity is not one.
fn quantityDim(q: Quantity) ?Dim {
    return switch (q) {
        .position => base.dim.length,
        .velocity => base.dim.speed,
        .acceleration => base.dim.acceleration,
        .force => base.dim.force,
        .moment => base.dim.moment,
        .angular_rate => base.dim.angular_rate,
        .scalar, .wrench, .quaternion => null,
    };
}

fn checkPortType(comptime name: []const u8, comptime T: type, comptime pt: PortType) void {
    const where = "port '" ++ name ++ "' of " ++ @typeName(T);
    if (pt.shape == .scalar) {
        if (pt.quantity != .scalar) {
            @compileError(where ++ ": a scalar port must have quantity .scalar");
        }
        if (pt.frame != .none) {
            @compileError(where ++ ": a scalar port cannot have a frame");
        }
        return;
    }
    if (pt.frame == .none) {
        @compileError(where ++ ": a " ++ @tagName(pt.shape) ++ " port must have a frame");
    }
    switch (pt.shape) {
        .quat => {
            if (pt.quantity != .quaternion) {
                @compileError(where ++ ": a quat port must have quantity .quaternion");
            }
        },
        .wrench => {
            if (pt.quantity != .wrench) {
                @compileError(where ++ ": a wrench port must have quantity .wrench");
            }
        },
        .vec3 => {
            if (pt.quantity == .quaternion or pt.quantity == .wrench) {
                @compileError(where ++ ": a vec3 port cannot have quantity " ++
                    @tagName(pt.quantity));
            }
            if (quantityDim(pt.quantity)) |expected| {
                if (!pt.dim.eql(expected)) {
                    @compileError(where ++ ": the dimension does not match its vector " ++
                        "quantity");
                }
            }
        },
        .scalar => unreachable,
    }
}

/// Runtime form of the checks in `checkPortType`, for port types that arrive as data (an
/// external signal) rather than as compile-time declarations.
pub fn isValidPortType(pt: PortType) bool {
    return switch (pt.shape) {
        .scalar => pt.quantity == .scalar and pt.frame == .none,
        .vec3 => pt.frame != .none and pt.quantity != .quaternion and
            pt.quantity != .wrench and
            (quantityDim(pt.quantity) == null or quantityDim(pt.quantity).?.eql(pt.dim)),
        .quat => pt.frame != .none and pt.quantity == .quaternion,
        .wrench => pt.frame != .none and pt.quantity == .wrench,
    };
}

fn readShape(comptime T: type, values: []const f64) T {
    if (T == f64) return values[0];
    if (T == base.Vec3) return .{ .x = values[0], .y = values[1], .z = values[2] };
    if (T == base.Quat) {
        return .{ .w = values[0], .x = values[1], .y = values[2], .z = values[3] };
    }
    if (T == base.Wrench) return .{
        .force = readShape(base.Vec3, values[0..3]),
        .moment = readShape(base.Vec3, values[3..6]),
    };
    @compileError("port field must be f64, Vec3, Quat or Wrench, not " ++ @typeName(T));
}

fn writeShape(comptime T: type, value: T, out: []f64) void {
    if (T == f64) {
        out[0] = value;
        return;
    }
    if (T == base.Vec3) {
        out[0] = value.x;
        out[1] = value.y;
        out[2] = value.z;
        return;
    }
    if (T == base.Quat) {
        out[0] = value.w;
        out[1] = value.x;
        out[2] = value.y;
        out[3] = value.z;
        return;
    }
    if (T == base.Wrench) {
        writeShape(base.Vec3, value.force, out[0..3]);
        writeShape(base.Vec3, value.moment, out[3..6]);
        return;
    }
    @compileError("port field must be f64, Vec3, Quat or Wrench, not " ++ @typeName(T));
}

pub fn fromSlice(comptime T: type, values: []const f64) T {
    const info = @typeInfo(T).@"struct";
    assert(values.len == slotCount(T));
    if (comptime info.field_names.len == 0) return .{};
    var result: T = undefined;
    var cursor: usize = 0;
    inline for (info.field_names, info.field_types) |name, FT| {
        @field(result, name) = readShape(FT, values[cursor..]);
        cursor += @as(usize, widthOf(FT));
    }
    return result;
}

pub fn toSlice(comptime T: type, value: T, out: []f64) void {
    const info = @typeInfo(T).@"struct";
    assert(out.len == slotCount(T));
    var cursor: usize = 0;
    inline for (info.field_names, info.field_types) |name, FT| {
        writeShape(FT, @field(value, name), out[cursor..]);
        cursor += @as(usize, widthOf(FT));
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
        const too_wide = sig.inputSlots() > limits.max_slots or
            sig.outputSlots() > limits.max_slots or sig.stateSlots() > limits.max_slots;
        if (too_wide) @compileError(@typeName(M) ++ " exceeds limits.max_slots");
        const stateful = sig.states.len > 0;
        if (stateful != @hasDecl(M, "derivatives")) {
            @compileError(@typeName(M) ++ ": `derivatives` must exist exactly when it has states");
        }
        if (!stateful and !sig.feedthrough) {
            @compileError(@typeName(M) ++ ": a stateless kind must have feedthrough = true");
        }
    }
}

/// Flat-slice form of `outputs`, used by the compiled plan. Slices are slot-sized.
pub fn outputsFlat(
    comptime M: type,
    model: *const M,
    x: []const f64,
    u: []const f64,
    y: []f64,
    ctx: Context,
) Validity {
    const r = model.outputs(fromSlice(M.State, x), fromSlice(M.In, u), ctx);
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
    ctx: Context,
) Validity {
    if (comptime M.signature.states.len == 0) {
        assert(dx.len == 0);
        return .valid;
    } else {
        const r = model.derivatives(fromSlice(M.State, x), fromSlice(M.In, u), ctx);
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
///   * evaluation is deterministic, bit for bit, for the same context;
///   * for kinds declared `feedthrough = false`, outputs ignore the inputs entirely
///     (verified by replacing the inputs with NaN).
/// The context is varied between samples so a kind that reads its placement is exercised,
/// but no equality across contexts is assumed: placement is legitimate per-instance data.
pub fn expectKindContract(
    comptime M: type,
    model: *const M,
    rng: *base.Rng,
    samples: u32,
) !void {
    const sig = M.signature;
    const ni = sig.inputSlots();
    const no = sig.outputSlots();
    const ns = sig.stateSlots();

    var x: [limits.max_slots]f64 = undefined;
    var u: [limits.max_slots]f64 = undefined;
    var y1: [limits.max_slots]f64 = undefined;
    var y2: [limits.max_slots]f64 = undefined;
    var y3: [limits.max_slots]f64 = undefined;
    var dx1: [limits.max_slots]f64 = undefined;
    var dx2: [limits.max_slots]f64 = undefined;

    for (0..samples) |_| {
        randomFill(x[0..ns], rng);
        randomFill(u[0..ni], rng);
        const ctx: Context = .{ .placement = .{
            .position = .{
                .x = rng.range(-5.0, 5.0),
                .y = rng.range(-5.0, 5.0),
                .z = rng.range(-5.0, 5.0),
            },
            .orientation = base.Quat.fromEuler(
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
                rng.range(-3.0, 3.0),
            ),
        } };

        const v1 = outputsFlat(M, model, x[0..ns], u[0..ni], y1[0..no], ctx);
        const v2 = outputsFlat(M, model, x[0..ns], u[0..ni], y2[0..no], ctx);
        for (y1[0..no]) |value| try std.testing.expect(std.math.isFinite(value));
        try std.testing.expectEqual(v1, v2);
        try std.testing.expect(sameBits(y1[0..no], y2[0..no]));

        // A different placement must still produce a finite, deterministic result.
        const ctx2: Context = .{ .placement = .{} };
        const v3 = outputsFlat(M, model, x[0..ns], u[0..ni], y3[0..no], ctx2);
        _ = v3;
        for (y3[0..no]) |value| try std.testing.expect(std.math.isFinite(value));

        if (!sig.feedthrough) {
            @memset(u[0..ni], std.math.nan(f64));
            _ = outputsFlat(M, model, x[0..ns], u[0..ni], y2[0..no], ctx);
            try std.testing.expect(sameBits(y1[0..no], y2[0..no]));
            randomFill(u[0..ni], rng);
        }

        if (ns > 0) {
            const d1 = derivativesFlat(M, model, x[0..ns], u[0..ni], dx1[0..ns], ctx);
            const d2 = derivativesFlat(M, model, x[0..ns], u[0..ni], dx2[0..ns], ctx);
            for (dx1[0..ns]) |value| try std.testing.expect(std.math.isFinite(value));
            try std.testing.expectEqual(d1, d2);
            try std.testing.expect(sameBits(dx1[0..ns], dx2[0..ns]));
        }
    }
}

// ---------------------------------------------------------------------------------------
// Tests of the contract machinery itself, on toy kinds.
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

        pub fn outputs(self: *const Model, x: State, u: In, ctx: Context) Evaluated(Out) {
            _ = self;
            _ = u;
            _ = ctx;
            return .{ .value = .{ .position = x.position } };
        }

        pub fn derivatives(self: *const Model, x: State, u: In, ctx: Context) Evaluated(State) {
            _ = ctx;
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
        .in = .{
            .force = scalar(base.dim.force),
            .offset = scalar(base.dim.length),
        },
        .in_sum = .{ .force = true },
        .out = .{ .position = scalar(base.dim.length) },
        .state = .{
            .position = scalar(base.dim.length),
            .velocity = scalar(base.dim.speed),
        },
        .feedthrough = false,
    });
};

/// A stateless kind whose ports use every wide shape: vec3 + quat in, wrench out.
const toy_wide = struct {
    const Model = union(enum) {
        passthrough: Pass,

        pub const In = struct { v: base.Vec3, attitude: base.Quat };
        pub const Out = struct { effort: base.Wrench };
        pub const State = struct {};

        pub const signature = contract_signature;

        pub fn validate(self: *const Model) ParamCheck {
            _ = self;
            return null;
        }

        pub fn outputs(self: *const Model, x: State, u: In, ctx: Context) Evaluated(Out) {
            _ = self;
            _ = x;
            _ = ctx;
            assertFiniteFields(u);
            return .{ .value = .{ .effort = .{
                .force = u.v,
                .moment = .{ .x = u.attitude.w, .y = u.attitude.x, .z = u.attitude.y },
            } } };
        }
    };

    const Pass = struct {};

    const contract_signature = signature(Model, .{
        .in = .{
            .v = vec3(base.dim.speed, .velocity, .body),
            .attitude = quat(.body),
        },
        .out = .{ .effort = wrench(.body) },
        .feedthrough = true,
    });
};

test "signature is built from the port structs" {
    const sig = toy.Model.signature;
    try std.testing.expectEqual(@as(usize, 2), sig.inputs.len);
    try std.testing.expectEqualStrings("force", sig.inputs[0].name);
    try std.testing.expectEqual(FanIn.sum, sig.inputs[0].ptype.fan_in);
    try std.testing.expectEqual(FanIn.single, sig.inputs[1].ptype.fan_in);
    try std.testing.expect(sig.inputs[0].ptype.dim.eql(base.dim.force));
    try std.testing.expectEqual(@as(?u32, 1), sig.inputIndex("offset"));
    try std.testing.expectEqual(@as(?u32, null), sig.inputIndex("nope"));
    try std.testing.expectEqual(@as(?u32, 1), sig.stateIndex("velocity"));
    try std.testing.expect(!sig.feedthrough);
    try std.testing.expectEqual(@as(u16, 2), sig.inputSlots());
    try std.testing.expectEqual(@as(u16, 1), sig.outputSlots());
    try std.testing.expectEqual(@as(u16, 2), sig.stateSlots());
    assertKind(toy.Model);
}

test "wide ports carry slots and round-trip through flat slices" {
    const Model = toy_wide.Model;
    const sig = Model.signature;
    try std.testing.expectEqual(@as(u16, 7), sig.inputSlots());
    try std.testing.expectEqual(@as(u16, 6), sig.outputSlots());
    try std.testing.expectEqual(@as(u16, 0), sig.stateSlots());
    try std.testing.expectEqual(@as(u8, 3), sig.inputs[0].ptype.width());
    try std.testing.expectEqual(@as(u8, 4), sig.inputs[1].ptype.width());
    try std.testing.expectEqual(@as(u8, 6), sig.outputs[0].ptype.width());
    try std.testing.expectEqual(@as(u16, 3), slotsBefore(sig.inputs, 1));
    try std.testing.expectEqual(@as(u16, 7), slotCount(Model.In));
    try std.testing.expectEqual(@as(u16, 6), slotCount(Model.Out));

    const u = [_]f64{ 1.0, 2.0, 3.0, 0.5, -0.5, 0.25, -0.75 };
    const typed = fromSlice(Model.In, &u);
    try std.testing.expectEqual(@as(f64, 2.0), typed.v.y);
    try std.testing.expectEqual(@as(f64, -0.75), typed.attitude.z);

    const m: Model = .{ .passthrough = .{} };
    var y: [6]f64 = undefined;
    try std.testing.expectEqual(
        Validity.valid,
        outputsFlat(Model, &m, &.{}, &u, &y, .{}),
    );
    const out = fromSlice(Model.Out, &y);
    try std.testing.expectEqual(@as(f64, 1.0), out.effort.force.x);
    try std.testing.expectEqual(@as(f64, 3.0), out.effort.force.z);
    try std.testing.expectEqual(@as(f64, 0.5), out.effort.moment.x);
    try std.testing.expectEqual(@as(f64, -0.5), out.effort.moment.y);
    var back: [6]f64 = undefined;
    toSlice(Model.Out, out, &back);
    try std.testing.expectEqualSlices(f64, &y, &back);
}

test "port compatibility rejects shape, quantity, frame and dimension" {
    const v_body = vec3(base.dim.speed, .velocity, .body);
    const v_world = vec3(base.dim.speed, .velocity, .world);
    const f_body = vec3(base.dim.force, .force, .body);
    const s = scalar(base.dim.speed);
    try std.testing.expectEqual(@as(?Incompatibility, null), compatibility(v_body, v_body));
    try std.testing.expectEqual(Incompatibility.shape, compatibility(s, v_body).?);
    try std.testing.expectEqual(Incompatibility.quantity, compatibility(v_body, f_body).?);
    try std.testing.expectEqual(Incompatibility.frame, compatibility(v_body, v_world).?);
    try std.testing.expectEqual(
        Incompatibility.dimension,
        compatibility(s, scalar(base.dim.force)).?,
    );
    // A wrench is exempt from the single-dimension check.
    try std.testing.expectEqual(
        @as(?Incompatibility, null),
        compatibility(wrench(.body), wrench(.body)),
    );
}

test "flat marshalling round-trips and matches the typed call" {
    const m: toy.Model = .{ .spring = .{ .k = 4.0, .mass = 2.0 } };
    const x = [_]f64{ 0.5, -1.0 };
    const u = [_]f64{ 10.0, 0.25 };
    var dx: [2]f64 = undefined;
    _ = derivativesFlat(toy.Model, &m, &x, &u, &dx, .{});
    const typed = m.derivatives(
        fromSlice(toy.Model.State, &x),
        fromSlice(toy.Model.In, &u),
        .{},
    );
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

test "the generic property test accepts a wide kind" {
    const m: toy_wide.Model = .{ .passthrough = .{} };
    var rng = base.Rng.init(2);
    try expectKindContract(toy_wide.Model, &m, &rng, 500);
    assertKind(toy_wide.Model);
}

test "assertFiniteFields recurses into wide values" {
    const ok: toy_wide.Model.In = .{
        .v = .{ .x = 1.0, .y = -2.0, .z = 0.0 },
        .attitude = .{ .w = 1.0 },
    };
    assertFiniteFields(ok);
    const wr: base.Wrench = .{ .force = .{ .x = 1.0 }, .moment = .{ .z = -3.0 } };
    assertFiniteFields(wr);
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
