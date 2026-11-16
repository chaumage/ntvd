# Design

## 1. Purpose and scope

Evaluate and size vehicle architectures built from components described by physical
equations, and (later) optimize them against criteria the engineer defines.

Three requirements drive everything:

1. **Fidelity must be able to grow.** A component starts as a two-parameter formula and later
   becomes a table of measurements or the result of a detailed simulation, without touching
   the rest of the vehicle.
2. **Dynamics must fit.** The first uses are steady (forces, balance), but time-domain
   behavior (stability, maneuvering, transients) must not require redesigning the core.
3. **Wrong answers must be hard to produce.** A sizing tool is trusted for decisions. It must
   reject malformed models, flag results outside a model's validity, and never return NaN
   as if it were a number.

Out of scope for the core: user interface, file formats, plotting. They are shells around it.

## 2. Layers

```
src/core/               the solver (built today)
  L3  solve        time integration; (planned) Newton, trim, algebraic loops
  L2  model        Spec -> compile -> Plan (+ Workspace)
  L1  components   contract + kinds (air, airflow, body, gain, lift_surface, wing, ...)
  L0  base         dim, units, geometry, validity, tables, linalg, rng, atmosphere, limits
```

Dependencies point down only. `build.zig` creates one Zig module per layer and passes each
only the lower layers it may use, so a violation does not compile. `test/tidy.zig` repeats the
rule as a check on import statements, and forbids relative imports across layers.

Frame, sign, orientation and placement conventions are in [`CONVENTIONS.md`](CONVENTIONS.md).
They are normative: a component author picks up its local axes from there. Its geometric sign
rows are pinned by tests in `src/core/base/geometry.zig`; the document says where the rest are
pinned as the corresponding phases land.

Planned above L3, each with its own directory under `src/`: `analysis/` (trim, sweeps,
missions), `optimize/`, `io/`, `cli/`, `gui/`, and eventually a C ABI. They are empty except
for a README until the solver work that motivates them lands.

## 3. Decisions

### D1. Components are plain `f64` functions; no generic scalar, no automatic differentiation

The first proposal made components generic over the scalar type so that forward-mode AD would
come for free. Zig has no operator overloading, so every equation would become method-call
chains (`a.mul(b).add(c)`), which defeats the point of readable, reviewable physics.

Instead, components are written as natural `f64` equations. Jacobians (for Newton, for
gradient-based optimization) come from finite differences in the solver layer. If a model
later needs exact derivatives, an optional `derivative` hook can be added per kind without
changing existing ones. Decide when a real need appears, with measurements of the cost.

### D2. Kinds and models: the fidelity ladder

A **kind** is a physical role with a fixed signature (a lift surface takes airspeed, angle of
attack and density, and returns lift and drag). A **model** is one implementation of a kind.
In code a kind is a `union(enum)` whose variants are its models:

```zig
.lift_surface = .{ .linear = .{ ... } }        // first estimate
.lift_surface = .{ .polar_table = .{ ... } }   // measured or simulated polars
```

All models of a kind share its signature, so changing the model never changes the wiring.
Fidelity is a *dimension of the data*, not a property of the architecture. Adding a model is
adding one variant and its tests; adding a kind is adding one file and one line in
`registry.zig` (see `ADDING_A_COMPONENT.md`).

Measured data enters through `base.interp` tables, validated at construction (finite, strictly
increasing axes). Outside the measured range a lookup clamps and flags the result
`beyond_envelope`; it never extrapolates silently.

### D3. The contract: signature, feedthrough, states

Each kind declares in a `signature`: inputs, outputs and continuous states, each with a
**port type**, and whether outputs depend algebraically on inputs (`feedthrough`).

A port type is a `Shape` (how many flat slots it occupies), a physical `Dim`, a `Quantity`
(what it physically carries) and a `Frame` (which frame a vector is expressed in):

| shape    | slots | field type | example quantity        | dimension           |
|----------|-------|------------|-------------------------|---------------------|
| `scalar` | 1     | `f64`      | `.scalar`               | any (e.g. speed)    |
| `vec3`   | 3     | `Vec3`     | `.velocity`, `.force`   | implied by quantity |
| `quat`   | 4     | `Quat`     | `.quaternion`           | none                |
| `wrench` | 6     | `Wrench`   | `.wrench`               | force + moment      |

The shape is derived from the field type; the other properties are authored with helpers:
`scalar(dim.speed)`, `vec3(dim.force, .force, .body)`, `quat(.body)`, `wrench(.local)`.
A scalar has no frame; every wide port must name one. A physical vector quantity fixes its
dimension, so a `velocity` port cannot be declared in metres. The compiler rejects a
connection whose ports disagree on shape, quantity, frame or dimension.

Wide ports occupy consecutive slots in the flat signal and state arrays. One port has one
wiring entry, carrying `width` slots; a summing input adds every source slot by slot. The
plan stores slot offsets and widths, so evaluation stays a flat loop.

Every component function also receives a `contract.Context`, which carries the component's
resolved `Placement`: where its local frame sits in the body frame and how it is turned. The
plan fills it in from the spec, so a kind never parses Euler angles or keeps its own copy. A
kind that only works in its own local frame ignores it.

- A **static** kind has no states; its outputs depend on its inputs (`feedthrough = true`).
- A **dynamic** kind has states with `dx/dt = f(x, u)`. If its outputs depend on the states
  only (`feedthrough = false`), it breaks algebraic loops. This is what lets
  `velocity -> drag -> force -> acceleration -> velocity` compile with no solver.

This is the extension point for dynamics: richer bodies, battery and motor transients,
actuators are all more states and more ports on the same contract.
`contract.expectKindContract` is a property test every model must pass (finite for any
finite input, deterministic, and no dependence on inputs when `feedthrough = false`).

Input ports are either `single` (exactly one source) or `sum` (any number, added; used for
forces so that every contribution is wired explicitly with its sign).

### D4. Spec -> compile -> Plan

- **Spec**: plain data, unvalidated; what a person, a file or an editor produces.
- **compile**: checks counts against limits, names, parameters, initial states, that every
  path resolves, that connected ports agree on **shape, quantity, frame and dimension**, that
  every `single` input has exactly one source, and orders components so feedthrough ones
  follow their sources. Every failure is an error code (stable, testable) plus a
  human-readable diagnostic ("`speed [m*s^-1] cannot drive body.force [m*kg*s^-2]`").
- **Plan**: immutable and fully resolved; evaluation is flat loops over indices with no
  lookups and no error paths.

A connection between different frames is accepted when both ends are `vec3`: compile inserts
a generated `frame_transform` node that rotates the vector into the destination frame. The
node is an ordinary component with a reserved name (`__x_<n>`), so it inherits evaluation,
validity and ordering. A `scalar` has no frame and cannot cross one. A world-frame transform
reads the designated body's `attitude`; if the plan has no such body, compilation fails.

A rigidly attached component is one `.rigid` wire from its local `wrench` output to the
designated body's `mount` input. Compile generates a single mount node
(`F_body = R F_local`, `M_body = R M_local + r × F_body`) from the source's placement, and
`body.mount` sums the contributions of every attached component. A component may attach at
most once; a second wire is `MultiplyDriven`.

Structural mistakes are therefore caught once, when the model is built, never in a hot loop.

### D5. Evaluation is total; validity is explicit

`Plan.evaluate/derivatives/rhs` accept any `f64` input, NaN and infinity included, and always
return finite signals plus a `Validity`:

- `valid`: inside the model's calibrated range
- `beyond_envelope`: finite result obtained by clamping (stall, end of a table, ISA ceiling)
- `invalid`: physically impossible input, or a non-finite value reached a component; outputs
  are zero and carry no meaning

Optimizers and solvers will probe nonsense. They get a flagged, finite answer they can
penalize, instead of a crash or a silent NaN.

Assertions inside components protect against *programmer* error (a component called directly
with a non-finite input). The plan guards the boundary so that operating conditions (a
diverging simulation, a wild optimizer step) never reach them. The integrator likewise never
returns a non-finite state: it fails with `error.Diverged` and keeps the last good one.

### D6. Dimensions are checked when the model is compiled, not by the type system

Every port carries a `Dim` (exponents of m, kg, s, K, A, plus radian as its own dimension, so
an angle cannot be confused with a plain ratio). `compile` rejects connections between unequal
dimensions. Compile-time dimensional types for *values* inside component code were considered
and rejected for now: they add friction to every equation for a class of error that the
port-level check already catches at the seams where it actually occurs. Revisit if internal
unit mistakes appear.

All values are SI inside the core. Conversion from knots, feet and the like happens at the
boundary (`base.units`).

### D7. Caller-provided exact storage

`src/core/base/limits.zig` keeps only the widths of the encoding (indices are `u16`; a port has
at most `max_ports` slots) and solver policies. There are no global capacities: the size of a
plan and a workspace are computed from the spec.

Nothing in `src/core/` allocates. A plan and a workspace are carved out of a byte buffer the
caller owns, aligned to `model.storage_alignment`:

```zig
const needed = model.bytesNeeded(&spec);           // exact for this spec
const plan = try model.compileInto(buffer, &spec, &diag);
const ws = model.Workspace.into(plan, ws_buffer);
```

`Plan` and `Workspace` are headers of slices into that buffer, so passing a plan by pointer is
cheap and evaluation touches only caller-owned memory. Undersized storage is refused up front
with `error.BufferTooSmall` and a diagnostic stating the size difference. Reusing memory means
recompiling into the same buffer, outside the solver.

### D8. Determinism

No wall-clock, no global state, no hash-map iteration order. The random generator is
implemented in `src/core/base/rng.zig` (xoshiro256++ seeded by SplitMix64) rather than taken
from `std`, because the standard library may change its generators between releases and
golden tests need a sequence that never moves. A test pins it to the standard library's
today. The same input gives bit-identical output, so every failure reproduces from a seed.

### D9. Algebraic loops are rejected for now

Feedthrough components feeding each other in a cycle are reported (`AlgebraicLoop`, naming
the components). Planned: route them to the nonlinear solver as unknowns. Until then, break
the loop with a dynamic component or turn the loop variable into an external that an analysis
solves for.

## 4. How fidelity evolves on a real project

1. Wire the vehicle with first-order models (`linear` wing, `linear_decay` thruster).
2. Size and explore: sweeps, trim, optimization on that cheap model.
3. A wind-tunnel or CFD campaign produces polars: load them into a `polar_table`, swap the
   one value. The connections, analyses and optimization setup are unchanged.
4. Compare the two models on the same architecture; the difference is the quantified cost
   of the early assumption.
5. Where a detailed solver is too slow to call directly, sample it into a table (or, later,
   a surrogate variant) and use that.

## 5. Testing

| Kind | Where | What it protects |
|---|---|---|
| Unit | each file | analytic cases, hand calculations, tables reproduce linear/bilinear data exactly |
| Contract property | `contract.expectKindContract` | every model of every kind: finite, deterministic, feedthrough honest |
| Scenario | `src/core/model/scenarios.zig` | physical loop compiles; model swap leaves wiring identical; totality under NaN/inf |
| Dynamics | `src/core/solve/integrate.zig` | RK4 vs analytic terminal-velocity solution; measured 4th-order convergence; divergence handling |
| Fuzz | `test/fuzz_model.zig` | random valid and invalid architectures: compile never crashes, plans are sound, evaluation is total; checks that its own generator reaches each error class |
| Style | `test/tidy.zig` | the rules in `TIGER_STYLE.md` |

The fuzzer has run 800 000 random architectures across several seeds in ReleaseSafe
(about 110 000 compiled into plans) without a failure.

## 6. Status and roadmap

Done: base layer; component contract with dynamics and fidelity variants; ten kinds
(`air`, `airflow`, `body`, `frame_transform`, `gain`, `lift_surface`, `load`, `point_mass`,
`thruster`, `wing`); spec/compile/plan; RK4 integration; fuzzer; style checker; frame/placement
conventions and geometry primitives (`Vec3`, `Quat`, `Rotation`); placements in the spec, a
component `Context`, generated cross-frame vector transforms, and single-wire rigid attachment
(Phase 3); the 6-DOF body with Newton-Euler dynamics, gravity, quaternion kinematics and
post-step normalization (Phase 4); local-frame aero with `airflow` stations, `wing` linear and
polar-table models, and thruster wrenches for rigid attachment (Phase 5).

Next, roughly in order:

1. **Newton solver and steady trim** (`solve`, `analysis`): residual form `F(x) = 0`, damped,
   bounded iterations, structured result (`converged`, `max_iter`, `singular`, `outside
   envelope`). Then route algebraic loops to it (D9).
2. **Sweeps and mission segments** (climb, cruise, foiling, range): energy and mass closure
   as property tests.
3. **Optimizer** (`optimize`): design variables bound to spec parameters by path, objectives
   and constraints over analysis outputs; Nelder-Mead and CMA-ES first, NSGA-II later.
   Infeasible or `beyond_envelope` points penalized, never NaN.
4. **More kinds** to prove the core is vehicle-agnostic: hull resistance, foil with ground and
   surface effect, motor, battery, aero moments from measured tables. The real test is that
   foils and hulls fit without touching `model` or `solve`.
5. **Model variants** beyond tables: 2D tables for propeller maps are in `base.interp` but not
   yet used by a kind; a surrogate variant; a bridge to external simulation results.
6. **I/O and C ABI**: JSON (strictly validated) for specs and results; a stable C interface,
   once the spec format has settled.

## 7. Known limitations

- The 6-DOF body models principal inertia only (no products of inertia). The generic
  cross-frame velocity transform still refuses a local destination; local flow comes from an
  `airflow` station, which applies the rate shift `ω × r`. `point_mass` remains the
  one-dimensional sizing kind.
- A `wing` has no intrinsic pitching moment: `wrench.moment` is zero. A measured moment model
  is a future extension. An airflow station and the surface it feeds must be given the same
  placement; the compiler cannot check that agreement.
- The set of kinds is closed (a tagged union): adding one means editing `registry.zig`.
  Third-party kinds would need a separate C-ABI variant.
- Time integration is fixed-step explicit RK4 with externals held constant over a step. Stiff
  systems and event handling (touchdown, takeoff) are not covered.
- Tables are piecewise linear (1D and 2D). Smoother interpolation can be added behind the same
  `Lookup` interface.
- Capacity limits are compile-time constants in `limits.zig`; raising them is a rebuild.
