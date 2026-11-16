# Conventions

Physical and geometric conventions for `ntvd`. Anything that crosses a frame boundary, a sign
or an angle order is written down here. The geometric rows of the sign table are pinned by tests
in `src/core/base/geometry.zig`; §4 says where the remaining rows are pinned.

## 1. Frames

All frames are **right-handed**. A positive rotation is counter-clockwise when looking from the
positive end of the axis towards the origin.

| Frame | Axes | Used by |
|---|---|---|
| **World (inertial)** | `x` north, `y` east, `z` **down** (NED) | externals, body position, gravity |
| **Body** | `x` forward, `y` right, `z` **down** | the vehicle's rigid body |
| **Local** | defined by each kind (see below) | a component's parameters and ports |

Gravity is the acceleration `(0, 0, +g)` in world coordinates; at level attitude the
gravitational force is `(0, 0, +m g)` in the body frame.

A component's **local** frame is its natural frame. Its parameters and its ports are expressed
there, never in world coordinates. Kinds define their axes in their module header; the first
ones are:

- **Wing / lifting surface:** `x` chordwise forward, `y` spanwise right, `z` normal down.
  Positive angle of attack gives lift along local `-z`; positive drag is along local `-x`;
  positive sideslip gives side force along local `+y`.
- **Thruster:** `x` along the thrust axis, positive forward.

## 2. Placements

A `ComponentSpec.placement` is **body-relative** and fixed for the whole run (relative motion
between components is out of scope for now):

- `position_m`: the origin of the local frame, in the body frame, metres.
- `attitude_rad`: intrinsic Z-Y-X `(roll, pitch, yaw)`, radians, giving **body ← local**.

The rotation matrix `R` maps local coordinates to the body frame: `p_body = R · p_local`.
A rigid attachment of a component to the vehicle transfers its local wrench to the body origin:

```
F_body = R · F_local
M_body = R · M_local + r_body × F_body
```

## 3. Orientation representation

- **Authoring and reporting:** intrinsic Z-Y-X Euler angles `(roll, pitch, yaw)`, radians.
- **Storage and computation:** unit quaternion `(w, x, y, z)`, Hamilton convention.
- `q` is the **pitch rate**; the orientation is always called `attitude` or `quat`. Never use
  `q` for a quaternion.
- `q` and `-q` denote the same rotation. Equality of orientations is up to sign.
- A quaternion state is renormalized after each accepted integration step. Its derivative is
  `attitude_dot = ½ attitude ⊗ (0, ω)`, with `ω` the body rate.

## 4. Sign table

Each row is an invariant. Geometric and rigid-body rows are pinned today by tests in
`src/core/base/geometry.zig`, `src/core/components/body.zig` and `src/core/model/compile.zig`;
aerodynamic rows are pinned by the Phase 5 tests in the implementation plan, and the table is
not complete until their tests exist.

| Operation | Expected result | Pinned by |
|---|---|---|
| positive `alpha` on a wing | lift along local `-z` | Phase 5 |
| positive `beta` on a wing | side force along local `+y` | Phase 5 |
| positive roll rate `p` | right wing down | `Quat: positive body rates match the sign table` |
| positive pitch rate `q` | nose up | `Quat: positive body rates match the sign table` |
| positive yaw rate `r` | nose right | `Quat: positive body rates match the sign table` |
| force `F = (1, 0, 0)` at `r = (0, 0, -0.4)` (above the CG, z is down) | moment `-y` (`r × F`), nose down | `compile: a rigid wrench at an offset` |
| gravity at level attitude | body force `(0, 0, +m g)` | `body: gravity is world-down and rotates into the body frame` |
| positive yaw of 90° | body `+x` maps to world `+y` | `Quat: identity and basis rotations` |
| positive pitch of 90° | body `+x` maps to world `-z` (up) | `Quat: identity and basis rotations` |
| positive roll of 90° | body `+y` maps to world `+z` (down) | `Quat: identity and basis rotations` |
| `toEuler(fromEuler(roll, pitch, yaw))` | `(roll, pitch, yaw)` for `\|pitch\| < 90°` | `Quat: Eulers round-trip` |

## 5. Rigid attachment

A component is attached to the vehicle with **one** connection:

```zig
.{ .from = "wing.wrench", .to = "body.mount", .attachment = .rigid },
```

- The source is a `wrench` output in the component's local frame.
- The destination is the designated body's wrench summing input, in the body frame.
- `compile` generates one mount node from the source's placement, applying the transform of
  §2. There is no implicit second link and no separate force/moment wiring. The node is
  ordinary and inspectable as `__x_<n>`; its `wrench` output is what drives `body.mount`.
- A component attaches at most once: a second `.rigid` wire from the same component is
  `MultiplyDriven`. Different components may all attach to the same mount, whose
  contributions add.
- A non-wrench source is `RigidAttachmentNotWrench`; a destination other than the body's mount
  is `RigidAttachmentToNonBody`; a rigid attachment with no designated body is `AmbiguousBody`.
- The connection `gain` is dimensionless and applies to the whole wrench; normally `1.0`.
  Reverse a sign in the component's local vectors, not in the attachment.

## 6. Body dynamics

A plan has at most one six-degree-of-freedom body. Its state order is velocity, rate, position,
attitude:

```
velocity_body (3)   rate_body (3)   position_world (3)   attitude (4)
```

Newton-Euler in the rotating body frame, with the origin at the CG and diagonal inertia:

```
g_body    = conj(attitude) · (0, 0, +g)
v_dot     = (F_mount + m g_body) / m - ω × v
ω_dot     = I⁻¹ (M_mount - ω × (I ω))
p_dot     = attitude · v
attitude_dot = ½ attitude · (0, ω)
```

- The body is not feedthrough: its outputs depend on the state alone, so reading the attitude
  cannot form an algebraic loop through the mount.
- `gravity_m_s2` defaults to `g0` and zero disables gravity. Only principal moments of inertia
  are modelled; products of inertia are a future extension.
- Acceleration transforms are refused entirely; velocity transforms involving a **local** frame
  are refused until the rate shift `ω × r` is implemented. The static transform of §2 is all
  that is applied today, together with pure rotations between world and body.

## 7. Reserved identifiers

- Generated nodes are named `__x_<n>`. User components and externals may not use the `__x_`
  prefix.
- SI everywhere inside the core. Unit conversion happens at the boundary (`base.units`).
