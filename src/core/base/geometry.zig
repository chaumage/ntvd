//! Geometry primitives: 3-vectors, unit quaternions and rotation matrices.
//!
//! Conventions are in `docs/CONVENTIONS.md`: world is NED, body is x forward / y right /
//! z down, and every frame is right-handed. A rotation maps its local frame to its parent
//! frame: `p_parent = R * p_local`.
//!
//! Quaternions are unit, stored `(w, x, y, z)` with the Hamilton product, and `q` and `-q`
//! denote the same rotation. Euler angles are intrinsic Z-Y-X `(roll, pitch, yaw)`, radians;
//! they are an authoring and reporting format only, never the internal state.
//!
//! Everything is a value type and allocation-free. A function that can fail returns `null`
//! rather than a NaN result; pure arithmetic states its precondition in the doc comment, and
//! preconditions that are cheap to check are asserted.

const std = @import("std");
const assert = std.debug.assert;

/// A 3-vector in whatever frame the caller is working in.
pub const Vec3 = struct {
    x: f64 = 0.0,
    y: f64 = 0.0,
    z: f64 = 0.0,

    pub const zero: Vec3 = .{};

    pub fn add(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };
    }

    pub fn sub(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };
    }

    pub fn scale(a: Vec3, k: f64) Vec3 {
        return .{ .x = a.x * k, .y = a.y * k, .z = a.z * k };
    }

    pub fn dot(a: Vec3, b: Vec3) f64 {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /// Right-hand rule: `x cross y = z`.
    pub fn cross(a: Vec3, b: Vec3) Vec3 {
        return .{
            .x = a.y * b.z - a.z * b.y,
            .y = a.z * b.x - a.x * b.z,
            .z = a.x * b.y - a.y * b.x,
        };
    }

    pub fn normSquared(a: Vec3) f64 {
        return a.dot(a);
    }

    pub fn norm(a: Vec3) f64 {
        return @sqrt(a.normSquared());
    }

    /// Unit vector, or `null` for a zero or non-finite vector.
    pub fn normalized(a: Vec3) ?Vec3 {
        const n2 = a.normSquared();
        if (!std.math.isFinite(n2) or n2 <= 0.0) return null;
        return a.scale(1.0 / @sqrt(n2));
    }

    pub fn isFinite(a: Vec3) bool {
        return std.math.isFinite(a.x) and std.math.isFinite(a.y) and std.math.isFinite(a.z);
    }
};

/// A force and a moment acting at one point, both in the same frame.
///
/// A wrench is the natural output of a component rigidly attached to a body: the attachment
/// transform only has to translate the moment by `r x F`. Flat port slots store `force`
/// xyz first, then `moment` xyz.
pub const Wrench = struct {
    force: Vec3 = Vec3.zero,
    moment: Vec3 = Vec3.zero,

    pub const zero: Wrench = .{};

    pub fn add(a: Wrench, b: Wrench) Wrench {
        return .{ .force = a.force.add(b.force), .moment = a.moment.add(b.moment) };
    }

    pub fn isFinite(self: Wrench) bool {
        return self.force.isFinite() and self.moment.isFinite();
    }
};

/// A rotation as a unit quaternion, Hamilton product, `(w, x, y, z)`.
pub const Quat = struct {
    w: f64 = 1.0,
    x: f64 = 0.0,
    y: f64 = 0.0,
    z: f64 = 0.0,

    pub const identity: Quat = .{};

    /// Hamilton product `a * b`. For column vectors, `R(a*b) = R(a) * R(b)`: `b` is applied
    /// first, then `a`.
    pub fn mul(a: Quat, b: Quat) Quat {
        return .{
            .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        };
    }

    pub fn conjugate(q: Quat) Quat {
        return .{ .w = q.w, .x = -q.x, .y = -q.y, .z = -q.z };
    }

    /// Unit quaternion, or `null` for a zero or non-finite one.
    pub fn normalized(q: Quat) ?Quat {
        const n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
        if (!std.math.isFinite(n2) or n2 <= 0.0) return null;
        const inv = 1.0 / @sqrt(n2);
        return .{ .w = q.w * inv, .x = q.x * inv, .y = q.y * inv, .z = q.z * inv };
    }

    /// Rotation of `angle` radians about `axis`, which is normalized internally.
    pub fn fromAxisAngle(axis: Vec3, angle: f64) ?Quat {
        const unit = axis.normalized() orelse return null;
        if (!std.math.isFinite(angle)) return null;
        const half = 0.5 * angle;
        const s = @sin(half);
        return .{ .w = @cos(half), .x = unit.x * s, .y = unit.y * s, .z = unit.z * s };
    }

    /// Intrinsic Z-Y-X: roll about local x, then pitch about local y, then yaw about local z.
    /// `R = Rz(yaw) * Ry(pitch) * Rx(roll)`.
    pub fn fromEuler(roll: f64, pitch: f64, yaw: f64) Quat {
        assert(std.math.isFinite(roll) and std.math.isFinite(pitch) and std.math.isFinite(yaw));
        const cr = @cos(0.5 * roll);
        const sr = @sin(0.5 * roll);
        const cp = @cos(0.5 * pitch);
        const sp = @sin(0.5 * pitch);
        const cy = @cos(0.5 * yaw);
        const sy = @sin(0.5 * yaw);
        const qx: Quat = .{ .w = cr, .x = sr };
        const qy: Quat = .{ .w = cp, .y = sp };
        const qz: Quat = .{ .w = cy, .z = sy };
        return qz.mul(qy).mul(qx);
    }

    /// Inverse of `fromEuler`, or `null` at gimbal lock (`|pitch|` near 90 degrees), where
    /// roll and yaw are not separable, or for a non-finite quaternion.
    pub fn toEuler(q: Quat) ?Vec3 {
        if (!q.isFinite()) return null;
        const r = q.toRotation();
        const sp = -r.m[2 * 3 + 0];
        if (!(@abs(sp) < 1.0 - 1.0e-12)) return null;
        const pitch = std.math.asin(sp);
        const roll = std.math.atan2(r.m[2 * 3 + 1], r.m[2 * 3 + 2]);
        const yaw = std.math.atan2(r.m[1 * 3 + 0], r.m[0 * 3 + 0]);
        return .{ .x = roll, .y = pitch, .z = yaw };
    }

    /// Rotation matrix of a unit quaternion, mapping local to parent coordinates.
    pub fn toRotation(q: Quat) Rotation {
        assert(q.isFinite());
        const xx = q.x * q.x;
        const yy = q.y * q.y;
        const zz = q.z * q.z;
        const xy = q.x * q.y;
        const xz = q.x * q.z;
        const yz = q.y * q.z;
        const wx = q.w * q.x;
        const wy = q.w * q.y;
        const wz = q.w * q.z;
        return .{ .m = .{
            1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy),
            2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
            2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy),
        } };
    }

    /// Rotates `v` from local to parent coordinates.
    pub fn rotate(q: Quat, v: Vec3) Vec3 {
        const u: Vec3 = .{ .x = q.x, .y = q.y, .z = q.z };
        const t = u.cross(v).scale(2.0);
        return v.add(t.scale(q.w)).add(u.cross(t));
    }

    /// Kinematics of a rotating body: `dq/dt = 0.5 * q * (0, omega)`, with `omega` the body
    /// rate expressed in the frame `q` maps from (body, for a body-to-parent rotation). The
    /// result is not unit; renormalize after the step.
    pub fn derivative(q: Quat, omega: Vec3) Quat {
        const dw = -q.x * omega.x - q.y * omega.y - q.z * omega.z;
        const dx = q.w * omega.x + q.y * omega.z - q.z * omega.y;
        const dy = q.w * omega.y - q.x * omega.z + q.z * omega.x;
        const dz = q.w * omega.z + q.x * omega.y - q.y * omega.x;
        return .{ .w = 0.5 * dw, .x = 0.5 * dx, .y = 0.5 * dy, .z = 0.5 * dz };
    }

    pub fn isFinite(q: Quat) bool {
        return std.math.isFinite(q.w) and std.math.isFinite(q.x) and
            std.math.isFinite(q.y) and std.math.isFinite(q.z);
    }
};

/// A resolved placement: where a component's local frame sits in the body frame.
///
/// `position` is the origin of the local frame in body coordinates, and `orientation` maps
/// local coordinates to body coordinates. The spec authors the orientation as intrinsic Z-Y-X
/// Euler angles; `compile` resolves it once to this value so evaluation never converts.
pub const Placement = struct {
    position: Vec3 = Vec3.zero,
    orientation: Quat = Quat.identity,

    pub fn isFinite(self: Placement) bool {
        return self.position.isFinite() and self.orientation.isFinite();
    }

    pub fn rotation(self: Placement) Rotation {
        return self.orientation.toRotation();
    }
};

/// A 3x3 rotation matrix, row-major, mapping local to parent coordinates.
pub const Rotation = struct {
    /// `m[row * 3 + col]`.
    m: [9]f64,

    pub const identity: Rotation = .{ .m = .{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    } };

    pub fn fromQuat(q: Quat) Rotation {
        return q.toRotation();
    }

    pub fn fromEuler(roll: f64, pitch: f64, yaw: f64) Rotation {
        return Quat.fromEuler(roll, pitch, yaw).toRotation();
    }

    pub fn apply(r: Rotation, v: Vec3) Vec3 {
        return .{
            .x = r.m[0] * v.x + r.m[1] * v.y + r.m[2] * v.z,
            .y = r.m[3] * v.x + r.m[4] * v.y + r.m[5] * v.z,
            .z = r.m[6] * v.x + r.m[7] * v.y + r.m[8] * v.z,
        };
    }

    pub fn transpose(r: Rotation) Rotation {
        return .{ .m = .{
            r.m[0], r.m[3], r.m[6],
            r.m[1], r.m[4], r.m[7],
            r.m[2], r.m[5], r.m[8],
        } };
    }

    pub fn mul(a: Rotation, b: Rotation) Rotation {
        var out: [9]f64 = undefined;
        for (0..3) |i| {
            for (0..3) |j| {
                var sum: f64 = 0.0;
                for (0..3) |k| sum += a.m[i * 3 + k] * b.m[k * 3 + j];
                out[i * 3 + j] = sum;
            }
        }
        return .{ .m = out };
    }

    pub fn isFinite(r: Rotation) bool {
        for (r.m) |value| {
            if (!std.math.isFinite(value)) return false;
        }
        return true;
    }
};

// ---------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------

const tolerance = 1.0e-12;

fn expectVec(expected: Vec3, actual: Vec3) !void {
    try std.testing.expectApproxEqAbs(expected.x, actual.x, tolerance);
    try std.testing.expectApproxEqAbs(expected.y, actual.y, tolerance);
    try std.testing.expectApproxEqAbs(expected.z, actual.z, tolerance);
}

fn expectRotationIdentity(r: Rotation) !void {
    const keys = [_]usize{ 0, 4, 8 };
    for (keys) |k| try std.testing.expectApproxEqAbs(@as(f64, 1.0), r.m[k], tolerance);
    for (r.m, 0..) |value, k| {
        if (k == 0 or k == 4 or k == 8) continue;
        try std.testing.expectApproxEqAbs(@as(f64, 0.0), value, tolerance);
    }
}

test "Vec3: cross product follows the right-hand rule" {
    const x: Vec3 = .{ .x = 1.0 };
    const y: Vec3 = .{ .y = 1.0 };
    const z: Vec3 = .{ .z = 1.0 };
    try expectVec(z, x.cross(y));
    try expectVec(x, y.cross(z));
    try expectVec(y, z.cross(x));
    try expectVec(z.scale(-1.0), y.cross(x));
}

test "Vec3: dot, cross and norm identities" {
    const a: Vec3 = .{ .x = 1.0, .y = 2.0, .z = -3.0 };
    const b: Vec3 = .{ .x = -4.0, .y = 5.0, .z = 6.0 };
    // (a x b) is perpendicular to both.
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), a.dot(a.cross(b)), tolerance);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), b.dot(a.cross(b)), tolerance);
    // Scalar triple product is cyclic.
    const c: Vec3 = .{ .x = 0.5, .y = -1.0, .z = 2.0 };
    const tv = a.dot(b.cross(c));
    try std.testing.expectApproxEqAbs(tv, b.dot(c.cross(a)), tolerance);
    try std.testing.expectApproxEqAbs(tv, a.cross(b).dot(c), tolerance);
    // |a x b| = |a||b| sin(theta); here a.b = 0 is not assumed, so check the identity.
    const lhs = a.cross(b).normSquared();
    const rhs = a.normSquared() * b.normSquared() - a.dot(b) * a.dot(b);
    try std.testing.expectApproxEqAbs(rhs, lhs, 1.0e-10);
}

test "Vec3: normalization rejects zero and non-finite input" {
    try expectVec(.{ .x = 1.0 }, (Vec3{ .x = 5.0 }).normalized().?);
    try std.testing.expect(Vec3.zero.normalized() == null);
    try std.testing.expect((Vec3{ .x = std.math.nan(f64) }).normalized() == null);
    try std.testing.expect((Vec3{ .y = std.math.inf(f64) }).normalized() == null);
    try std.testing.expect(!(Vec3{ .z = std.math.nan(f64) }).isFinite());
}

test "Quat: identity and basis rotations match the sign table" {
    const x: Vec3 = .{ .x = 1.0 };
    const y: Vec3 = .{ .y = 1.0 };
    const z: Vec3 = .{ .z = 1.0 };

    try expectVec(x, Quat.identity.rotate(x));

    // Positive yaw 90 degrees: body +x to world +y.
    const yaw = Quat.fromEuler(0.0, 0.0, std.math.pi / 2.0);
    try expectVec(y, yaw.rotate(x));

    // Positive pitch 90 degrees: body +x to world -z (up, because world z is down).
    const pitch = Quat.fromEuler(0.0, std.math.pi / 2.0, 0.0);
    try expectVec(z.scale(-1.0), pitch.rotate(x));

    // Positive roll 90 degrees: body +y to world +z (right wing down).
    const roll = Quat.fromEuler(std.math.pi / 2.0, 0.0, 0.0);
    try expectVec(z, roll.rotate(y));

    // A rotation about z by 90 degrees, built the other way, agrees.
    const axis = Quat.fromAxisAngle(z, std.math.pi / 2.0).?;
    try expectVec(y, axis.rotate(x));
}

test "Quat: q and -q are the same rotation" {
    const q = Quat.fromEuler(0.3, -0.4, 1.2);
    const minus: Quat = .{ .w = -q.w, .x = -q.x, .y = -q.y, .z = -q.z };
    const v: Vec3 = .{ .x = 1.0, .y = 2.0, .z = 3.0 };
    try expectVec(q.rotate(v), minus.rotate(v));
}

test "Quat: Eulers round-trip away from gimbal lock" {
    const eulers = [_]Vec3{
        .{},
        .{ .x = 0.2, .y = -0.3, .z = 1.1 },
        .{ .x = -1.2, .y = 0.8, .z = -2.0 },
        .{ .x = 2.9, .y = -1.4, .z = 0.1 },
    };
    for (eulers) |e| {
        const q = Quat.fromEuler(e.x, e.y, e.z);
        const back = Quat.toEuler(q).?;
        try std.testing.expectApproxEqAbs(e.x, back.x, 1.0e-10);
        try std.testing.expectApproxEqAbs(e.y, back.y, 1.0e-10);
        try std.testing.expectApproxEqAbs(e.z, back.z, 1.0e-10);
    }
    // At gimbal lock the extraction is not unique; refuse rather than lie.
    try std.testing.expect(Quat.toEuler(Quat.fromEuler(0.0, std.math.pi / 2.0, 0.0)) == null);
}

test "Quat: fromAxisAngle normalizes the axis and rejects bad input" {
    const axis: Vec3 = .{ .x = 0.0, .y = 0.0, .z = 10.0 };
    const q = Quat.fromAxisAngle(axis, std.math.pi).?;
    try expectVec(.{ .x = -1.0 }, q.rotate(.{ .x = 1.0 }));
    try std.testing.expect(Quat.fromAxisAngle(Vec3.zero, 1.0) == null);
    try std.testing.expect(Quat.fromAxisAngle(.{ .z = 1.0 }, std.math.inf(f64)) == null);
    try std.testing.expect(Quat.normalized(.{ .w = 0.0 }) == null);
    try std.testing.expect(Quat.normalized(.{ .x = std.math.nan(f64) }) == null);
    try std.testing.expect(Quat.toEuler(.{ .w = std.math.inf(f64) }) == null);
    try std.testing.expect(Quat.toEuler(.{ .y = std.math.nan(f64) }) == null);
}

test "Rotation: orthonormal, determinant +1, transpose is inverse" {
    const r = Rotation.fromEuler(0.4, -0.7, 2.1);
    const identity = r.mul(r.transpose());
    try expectRotationIdentity(identity);
    try expectRotationIdentity(r.transpose().mul(r));

    const det = r.m[0] * (r.m[4] * r.m[8] - r.m[5] * r.m[7]) -
        r.m[1] * (r.m[3] * r.m[8] - r.m[5] * r.m[6]) +
        r.m[2] * (r.m[3] * r.m[7] - r.m[4] * r.m[6]);
    try std.testing.expectApproxEqAbs(@as(f64, 1.0), det, 1.0e-12);

    const v: Vec3 = .{ .x = 1.0, .y = -2.0, .z = 0.5 };
    try expectVec(v, r.transpose().apply(r.apply(v)));
    try expectVec(v, r.apply(r.transpose().apply(v)));
}

test "Rotation: matrix composition matches quaternion composition" {
    const q1 = Quat.fromEuler(0.1, 0.2, 0.3);
    const q2 = Quat.fromEuler(-0.4, 0.5, 0.6);
    const composed = q1.mul(q2).toRotation();
    const matrices = q1.toRotation().mul(q2.toRotation());
    for (composed.m, matrices.m) |a, b| {
        try std.testing.expectApproxEqAbs(b, a, 1.0e-12);
    }
}

test "Quat: derivative of a pure roll rate" {
    // 0.5 * (1,0,0,0) * (0, p, 0, 0) = (0, p/2, 0, 0).
    const d = Quat.identity.derivative(.{ .x = 0.5 });
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), d.w, tolerance);
    try std.testing.expectApproxEqAbs(@as(f64, 0.25), d.x, tolerance);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), d.y, tolerance);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), d.z, tolerance);
}

test "Quat: integrating a yaw rate gives the analytic rotation" {
    // One radian per second for pi seconds is a 180 degree yaw.
    const omega: Vec3 = .{ .z = 1.0 };
    const dt = 1.0e-4;
    var q = Quat.identity;
    var t: f64 = 0.0;
    while (t < std.math.pi) : (t += dt) {
        const d = Quat.derivative(q, omega);
        q = q.normalized().?;
        q.w += dt * d.w;
        q.x += dt * d.x;
        q.y += dt * d.y;
        q.z += dt * d.z;
    }
    q = q.normalized().?;
    // Explicit Euler, so the angle carries an O(dt) error: 1e-5 is tight enough to prove the
    // convention and loose enough not to test the integrator.
    const v = q.rotate(.{ .x = 1.0 });
    try std.testing.expectApproxEqAbs(@as(f64, -1.0), v.x, 1.0e-5);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), v.y, 1.0e-5);
    try std.testing.expectApproxEqAbs(@as(f64, 0.0), v.z, 1.0e-5);
}

test "Quat: renormalization keeps the norm bounded over a long run" {
    var q = Quat.fromEuler(0.3, -0.2, 0.1);
    const omega: Vec3 = .{ .x = 0.1, .y = -0.2, .z = 0.3 };
    const dt = 1.0e-4;
    for (0..1_000_000) |_| {
        const d = Quat.derivative(q, omega);
        q.w += dt * d.w;
        q.x += dt * d.x;
        q.y += dt * d.y;
        q.z += dt * d.z;
        q = q.normalized().?;
    }
    const n = @sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    try std.testing.expectApproxEqAbs(@as(f64, 1.0), n, 1.0e-12);
    try std.testing.expect(q.isFinite());
}

test "Wrench: zero, add and finite check" {
    const a: Wrench = .{ .force = .{ .x = 1.0, .y = 2.0 }, .moment = .{ .z = -3.0 } };
    const b: Wrench = .{ .force = .{ .x = -1.0, .z = 4.0 }, .moment = .{ .z = 3.0 } };
    const sum = a.add(b);
    try expectVec(.{ .y = 2.0, .z = 4.0 }, sum.force);
    try expectVec(.{}, sum.moment);
    try std.testing.expect(Wrench.zero.isFinite());
    try std.testing.expect(a.isFinite());
    const bad: Wrench = .{ .moment = .{ .y = std.math.inf(f64) } };
    try std.testing.expect(!bad.isFinite());
}

test "geometry types have stable sizes" {
    try std.testing.expect(@sizeOf(Vec3) == 24);
    try std.testing.expect(@sizeOf(Wrench) == 48);
    try std.testing.expect(@sizeOf(Quat) == 32);
    try std.testing.expect(@sizeOf(Rotation) == 72);
    try std.testing.expect(Rotation.identity.isFinite());
}
