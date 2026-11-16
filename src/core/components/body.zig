//! Body: the vehicle as a six-degree-of-freedom rigid body.
//!
//! Input   mount [N, N*m] in body coordinates (summing); read by the dynamics, not by
//!         `outputs`
//! Outputs velocity_body, rate_body, position_world, attitude, velocity_world
//! State   velocity_body, rate_body, position_world, attitude (13 slots)
//!
//! The state is ordered velocity, rate, position, attitude. `attitude` is a unit quaternion
//! mapping body to world; the integrator renormalizes it after every accepted step.
//!
//! Newton-Euler in the rotating body frame, with the body origin at the CG:
//!
//!   g_body   = conj(attitude) * (0, 0, +g)
//!   v_dot    = (F_mount + m * g_body) / m - omega x v
//!   omega_dot = I^-1 * (M_mount - omega x (I * omega))
//!   p_dot    = attitude * v
//!   q_dot    = 0.5 * attitude * (0, omega)
//!
//! `I` is diagonal in the body frame: the body axes are the principal axes. Products of
//! inertia are a future extension.
//!
//! The kind is deliberately `feedthrough = false`: every output comes from the state alone,
//! so world-frame transforms can read the attitude without an algebraic loop through the
//! mount point. The compiler identifies the designated body by the presence of the wrench
//! `mount` input; a plan with more than one such component is ambiguous.

const std = @import("std");
const assert = std.debug.assert;
const base = @import("base");
const contract = @import("contract.zig");

pub const Model = union(enum) {
    six_dof: SixDof,

    pub const In = struct { mount: base.Wrench = .{} };

    pub const Out = struct {
        velocity_body: base.Vec3 = .{},
        rate_body: base.Vec3 = .{},
        position_world: base.Vec3 = .{},
        attitude: base.Quat = .{},
        velocity_world: base.Vec3 = .{},
    };

    pub const State = struct {
        velocity_body: base.Vec3 = .{},
        rate_body: base.Vec3 = .{},
        position_world: base.Vec3 = .{},
        attitude: base.Quat = .{},
    };

    pub const signature = contract.signature(Model, .{
        .in = .{ .mount = contract.wrench(.body) },
        .in_sum = .{ .mount = true },
        .out = .{
            .velocity_body = contract.vec3(base.dim.speed, .velocity, .body),
            .rate_body = contract.vec3(base.dim.angular_rate, .angular_rate, .body),
            .position_world = contract.vec3(base.dim.length, .position, .world),
            .attitude = contract.quat(.world),
            .velocity_world = contract.vec3(base.dim.speed, .velocity, .world),
        },
        .state = .{
            .velocity_body = contract.vec3(base.dim.speed, .velocity, .body),
            .rate_body = contract.vec3(base.dim.angular_rate, .angular_rate, .body),
            .position_world = contract.vec3(base.dim.length, .position, .world),
            .attitude = contract.quat(.world),
        },
        .feedthrough = false,
    });

    pub fn validate(self: *const Model) contract.ParamCheck {
        return switch (self.*) {
            inline else => |*m| m.validate(),
        };
    }

    pub fn outputs(
        self: *const Model,
        x: State,
        u: In,
        ctx: contract.Context,
    ) contract.Evaluated(Out) {
        return switch (self.*) {
            inline else => |*m| m.outputs(x, u, ctx),
        };
    }

    pub fn derivatives(
        self: *const Model,
        x: State,
        u: In,
        ctx: contract.Context,
    ) contract.Evaluated(State) {
        return switch (self.*) {
            inline else => |*m| m.derivatives(x, u, ctx),
        };
    }
};

/// Slots occupied by `Model.State`; useful to size an initial-state array.
pub const state_slots: usize = contract.slotCount(Model.State);

/// Serializes a named state in signature order, for `ComponentSpec.initial_state`.
pub fn initialState(state: Model.State) [state_slots]f64 {
    var out: [state_slots]f64 = undefined;
    contract.toSlice(Model.State, state, &out);
    return out;
}

pub const SixDof = struct {
    mass_kg: f64,
    /// Principal moments of inertia about the body axes.
    inertia_kg_m2: base.Vec3,
    /// Gravity acceleration along world +z (down). Zero disables gravity.
    gravity_m_s2: f64 = base.units.g0,

    pub fn validate(self: *const SixDof) contract.ParamCheck {
        if (!contract.isPositive(self.mass_kg)) return "mass_kg must be positive";
        if (!positiveVec(self.inertia_kg_m2)) {
            return "inertia_kg_m2 must be positive on every axis";
        }
        if (!contract.isNonNegative(self.gravity_m_s2)) {
            return "gravity_m_s2 must be non-negative";
        }
        return null;
    }

    /// Does not read the mount: this is what `feedthrough = false` promises.
    pub fn outputs(
        self: *const SixDof,
        x: Model.State,
        u: Model.In,
        ctx: contract.Context,
    ) contract.Evaluated(Model.Out) {
        _ = self;
        _ = u;
        _ = ctx;
        return .{ .value = .{
            .velocity_body = x.velocity_body,
            .rate_body = x.rate_body,
            .position_world = x.position_world,
            .attitude = x.attitude,
            .velocity_world = x.attitude.rotate(x.velocity_body),
        } };
    }

    pub fn derivatives(
        self: *const SixDof,
        x: Model.State,
        u: Model.In,
        ctx: contract.Context,
    ) contract.Evaluated(Model.State) {
        _ = ctx;
        contract.assertFiniteFields(x);
        contract.assertFiniteFields(u);
        assert(contract.isPositive(self.mass_kg));
        assert(positiveVec(self.inertia_kg_m2));
        assert(contract.isNonNegative(self.gravity_m_s2));

        const inertia = self.inertia_kg_m2;
        const omega = x.rate_body;
        const gravity_body = x.attitude.conjugate().rotate(.{ .z = self.gravity_m_s2 });
        const force = u.mount.force.add(gravity_body.scale(self.mass_kg));
        const velocity_dot = force.scale(1.0 / self.mass_kg)
            .sub(omega.cross(x.velocity_body));
        const h: base.Vec3 = .{
            .x = inertia.x * omega.x,
            .y = inertia.y * omega.y,
            .z = inertia.z * omega.z,
        };
        const moment = u.mount.moment.sub(omega.cross(h));
        const rate_dot: base.Vec3 = .{
            .x = moment.x / inertia.x,
            .y = moment.y / inertia.y,
            .z = moment.z / inertia.z,
        };
        return .{ .value = .{
            .velocity_body = velocity_dot,
            .rate_body = rate_dot,
            .position_world = x.attitude.rotate(x.velocity_body),
            .attitude = base.Quat.derivative(x.attitude, omega),
        } };
    }
};

fn positiveVec(v: base.Vec3) bool {
    return contract.isPositive(v.x) and contract.isPositive(v.y) and
        contract.isPositive(v.z);
}

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

fn expectVec(expected: base.Vec3, actual: base.Vec3) !void {
    try std.testing.expectApproxEqAbs(expected.x, actual.x, 1.0e-12);
    try std.testing.expectApproxEqAbs(expected.y, actual.y, 1.0e-12);
    try std.testing.expectApproxEqAbs(expected.z, actual.z, 1.0e-12);
}

fn expectQuat(expected: base.Quat, actual: base.Quat) !void {
    try std.testing.expectApproxEqAbs(expected.w, actual.w, 1.0e-12);
    try std.testing.expectApproxEqAbs(expected.x, actual.x, 1.0e-12);
    try std.testing.expectApproxEqAbs(expected.y, actual.y, 1.0e-12);
    try std.testing.expectApproxEqAbs(expected.z, actual.z, 1.0e-12);
}

test "the outputs are the states and the mount is ignored" {
    const m: Model = .{ .six_dof = .{
        .mass_kg = 2.0,
        .inertia_kg_m2 = .{ .x = 1.0, .y = 2.0, .z = 3.0 },
    } };
    const q = base.Quat.fromEuler(0.2, -0.1, 0.5);
    const v: base.Vec3 = .{ .x = 1.0, .y = 2.0, .z = 3.0 };
    const x: Model.State = .{
        .velocity_body = v,
        .rate_body = .{ .x = -1.0, .y = -2.0, .z = -3.0 },
        .position_world = .{ .x = 4.0, .y = 5.0, .z = 6.0 },
        .attitude = q,
    };
    const r = m.outputs(x, .{ .mount = .{ .force = .{ .x = 1.0 } } }, .{});
    try expectVec(v, r.value.velocity_body);
    try expectVec(x.rate_body, r.value.rate_body);
    try expectVec(x.position_world, r.value.position_world);
    try expectQuat(q, r.value.attitude);
    try expectVec(q.rotate(v), r.value.velocity_world);
    try std.testing.expectEqual(base.Validity.valid, r.validity);
}

test "newton-euler with rate coupling" {
    const m: Model = .{ .six_dof = .{
        .mass_kg = 2.0,
        .inertia_kg_m2 = .{ .x = 1.0, .y = 2.0, .z = 4.0 },
        .gravity_m_s2 = 0.0,
    } };
    const x: Model.State = .{
        .velocity_body = .{ .x = 1.0 },
        .rate_body = .{ .y = 1.0 },
    };
    const u: Model.In = .{ .mount = .{
        .force = .{ .x = 2.0 },
        .moment = .{ .y = 3.0 },
    } };
    const d = m.derivatives(x, u, .{});
    // F/m = (1,0,0); omega x v = (0,0,-1), so v_dot = (1,0,1).
    try expectVec(.{ .x = 1.0, .z = 1.0 }, d.value.velocity_body);
    // I*omega = (0,2,0); omega x I*omega = 0, so omega_dot = M/I = (0,1.5,0).
    try expectVec(.{ .y = 1.5 }, d.value.rate_body);
    try expectVec(.{ .x = 1.0 }, d.value.position_world);
    try expectQuat(.{ .w = 0.0, .y = 0.5 }, d.value.attitude);
    try std.testing.expectEqual(base.Validity.valid, d.validity);
}

test "gravity is world-down and rotates into the body frame" {
    const m: Model = .{ .six_dof = .{
        .mass_kg = 2.0,
        .inertia_kg_m2 = .{ .x = 1.0, .y = 1.0, .z = 1.0 },
    } };
    const level = m.derivatives(.{}, .{}, .{});
    try expectVec(.{ .z = base.units.g0 }, level.value.velocity_body);

    // Pitch 90 degrees: body +x points up, so world +z becomes body -x.
    const pitched = m.derivatives(
        .{ .attitude = base.Quat.fromEuler(0.0, std.math.pi / 2.0, 0.0) },
        .{},
        .{},
    );
    try expectVec(.{ .x = -base.units.g0 }, pitched.value.velocity_body);
}

test "the signature has 13 states, 16 outputs and a summing mount" {
    const sig = Model.signature;
    try std.testing.expectEqual(@as(u16, 6), sig.inputSlots());
    try std.testing.expectEqual(@as(u16, 16), sig.outputSlots());
    try std.testing.expectEqual(@as(u16, 13), sig.stateSlots());
    try std.testing.expectEqual(contract.FanIn.sum, sig.inputs[0].ptype.fan_in);
    try std.testing.expect(!sig.feedthrough);
    try std.testing.expectEqual(contract.Quantity.quaternion, sig.states[3].ptype.quantity);
}

test "initialState serializes in signature order" {
    const s: Model.State = .{
        .velocity_body = .{ .x = 1.0, .y = 2.0, .z = 3.0 },
        .rate_body = .{ .x = 4.0, .y = 5.0, .z = 6.0 },
        .position_world = .{ .x = 7.0, .y = 8.0, .z = 9.0 },
        .attitude = .{ .w = 10.0, .x = 11.0, .y = 12.0, .z = 13.0 },
    };
    const actual = initialState(s);
    const expected = [_]f64{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };
    try std.testing.expectEqualSlices(f64, &expected, &actual);
}

test "body parameters are validated" {
    const ok: Model = .{ .six_dof = .{
        .mass_kg = 1.0,
        .inertia_kg_m2 = .{ .x = 1.0, .y = 2.0, .z = 3.0 },
    } };
    try std.testing.expectEqual(@as(contract.ParamCheck, null), ok.validate());

    const zero_mass: Model = .{ .six_dof = .{
        .mass_kg = 0.0,
        .inertia_kg_m2 = .{ .x = 1.0, .y = 2.0, .z = 3.0 },
    } };
    try std.testing.expect(zero_mass.validate() != null);

    const zero_inertia: Model = .{ .six_dof = .{
        .mass_kg = 1.0,
        .inertia_kg_m2 = .{ .x = 1.0, .y = 0.0, .z = 3.0 },
    } };
    try std.testing.expect(zero_inertia.validate() != null);

    const negative_gravity: Model = .{ .six_dof = .{
        .mass_kg = 1.0,
        .inertia_kg_m2 = .{ .x = 1.0, .y = 2.0, .z = 3.0 },
        .gravity_m_s2 = -1.0,
    } };
    try std.testing.expect(negative_gravity.validate() != null);
}

test "body honors the component contract" {
    contract.assertKind(Model);
    const m: Model = .{ .six_dof = .{
        .mass_kg = 12.0,
        .inertia_kg_m2 = .{ .x = 1.0, .y = 2.0, .z = 3.0 },
    } };
    var rng = base.Rng.init(72);
    try contract.expectKindContract(Model, &m, &rng, 2000);
}
