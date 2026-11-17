# Adding to the physics library

Two operations: add a **model** to an existing kind (a new fidelity level), or add a new
**kind** (a new physical role). The first is the common one.

## A. Add a model to an existing kind

Example: a lift surface whose polars come from a lifting-line solver, as a closed form with
different parameters. Say it is called `swept_linear`.

1. In the kind's file (`src/core/components/lift_surface.zig`), define the model struct with its
   parameters and three functions:
   ```zig
   pub const SweptLinear = struct {
       // parameters, SI units, units in the names
       pub fn validate(self: *const SweptLinear) contract.ParamCheck { ... }
       pub fn outputs(
           self: *const SweptLinear,
           x: Model.State,
           u: Model.In,
           ctx: contract.Context,
       ) contract.Evaluated(Model.Out) { ... }
   };
   ```
2. Add it as a variant of the kind's `Model` union, and a branch in `validate`/`outputs`
   (the `inline else` already covers it).
3. Document at the top of the file: the local frame's axes, the equations, the validity
   range, and what is returned outside it. The compiler supplies the component's placement
   through `ctx`; the kind states its axes and parameters in that local frame.
4. Tests, in the same file:
   - a hand-calculated case (arithmetic in a comment)
   - the scaling/monotonicity property that physics demands
   - validity on both sides of the envelope (`valid`, `beyond_envelope`, `invalid`)
   - parameter validation (each rejected parameter, and one accepted set)
   - add it to the list in the "honors the component contract" test
5. If it replaces a cheaper model, add a test showing the two agree where they should (see
   "a table sampled from the analytic model gives nearly the same answer").

Nothing outside this file changes: wiring, analyses and the registry are untouched. That is
the check that the abstraction is right.

## B. Add a new kind

Example: `hull_resistance` (speed, density -> drag).

1. Create `src/core/components/hull_resistance.zig` with the file header comment, then:
   ```zig
   pub const Model = union(enum) {
       viscous_form: ViscousForm,

       pub const In = struct { speed: f64, density: f64 };
       pub const Out = struct { drag: f64 };
       pub const State = struct {};          // empty for a static kind

       pub const signature = contract.signature(Model, .{
           .in = .{
               .speed = contract.scalar(base.dim.speed),
               .density = contract.scalar(base.dim.density),
           },
           .out = .{ .drag = contract.scalar(base.dim.force) },
           .feedthrough = true,              // must be true when there are no states
       });
       pub fn validate(...) ...
       pub fn outputs(...) ...
       // pub fn derivatives(...)  only if State has fields
   };
   ```
   Every field of `In`, `Out` and `State` must be `f64`, `Vec3`, `Quat` or `Wrench` and
   must appear in the matching table; the compiler tells you precisely what is missing.
   Use `contract.vec3(dim, quantity, frame)`, `contract.quat(frame)` and
   `contract.wrench(frame)` for wide ports: the shape is derived from the field type and
   checked against the declared one. Mark inputs that add up (forces) with
   `.in_sum = .{ .force = true }`.
   `outputs` and `derivatives` take a `contract.Context` as their last argument. It carries
   the placement the compiler resolved for this component (its local frame's position and
   orientation in the body frame); a kind that works purely in its own frame ignores it.
   A kind that reads a world or body-frame signal is wired through generated transform nodes,
   not by doing frame math itself.
2. Register it in `src/core/components/registry.zig` (one import, one union field) and in
   `src/core/components/root.zig`. The compiler then points at every place that must learn about
   it, and `assertKind` checks the contract at compile time.
3. Write the tests listed in section A, including `expectKindContract` for every model.

## C. A dynamic kind

Give `State` fields, give `derivatives`, and decide `feedthrough`:

- `feedthrough = false` if outputs depend on states only (a mass, a battery's charge, an
  actuator position). This is what allows loops with the rest of the vehicle without a solver.
- `feedthrough = true` if some output also depends algebraically on an input.

`outputs` of a non-feedthrough kind must not read its input: the contract test checks this by
replacing the inputs with NaN.

## D. Declare a local frame and a placement

Every component works in its own local frame, and its axes are part of the kind's contract. At
the top of the file state those axes explicitly: for an aero surface, x chord-forward,
y span-right, z normal-down. Parameters such as areas and offsets are local; do not fold a
vehicle-level position into them.

The author supplies `ComponentSpec.placement` once per instance: `position_m` is the origin of
the local frame in the body frame, and `attitude_rad` is its intrinsic Z-Y-X orientation
relative to the body. The compiler resolves it into `contract.Context.placement`, which `outputs`
receives. There is no second place to write a position or an orientation. Components that
consume a station's flow must be given the **same placement** as that station; the compiler
cannot detect a mismatch, so pin it in a scenario test.

Never write frame math inside a kind. A connection whose source and target frames differ is
lowered to a generated `frame_transform` node, which is an ordinary node in the plan. The
exception is a rigid attachment: the compiler derives the mount transform from the source's
placement for a `.rigid` wire to `body.mount`.

## E. Output vectors, not world coordinates

A kind reports physical quantities in its declared frame, never a global position. Use
`contract.vec3(dim, quantity, frame)` and `contract.wrench(frame)` for wide ports; the shape
comes from the field type (`Vec3`, `Quat`, `Wrench`) and is checked against the declaration.
Body state enters only through the `body` kind, and world conditions enter as externals. An
`airflow` station, for example, takes body-frame velocity and rate and reports local `alpha`,
`beta` and `airspeed`; a `wing` takes those scalars and outputs a **local** `wrench`. Neither
knows where the aircraft is pointed.

To deliver a force to the vehicle, wire the local wrench to `body.mount` once with
`.attachment = .rigid`. The compiler inserts the rotation and the `r × F` moment. A `vec3`
cross-frame signal uses a plain `.signal` wire and is rotated automatically; note that velocity
transforms involving a local frame are rejected because they need the `ω × r` rate shift, which
an `airflow` station handles instead.

## Checklist before merging

- [ ] `zig build test` passes in Debug and with `-Doptimize=ReleaseSafe`
- [ ] `zig fmt --check build.zig src test` is clean
- [ ] file header documents the local frame's axes, equations, validity range, behavior
      outside it
- [ ] no allocation, no recursion, every loop bounded
- [ ] names avoid the `__x_` prefix, which is reserved for generated transform nodes
- [ ] local axes and the placement rule are documented, with a test for a shared station
- [ ] wide ports declare a frame and output local vectors, not world coordinates
- [ ] a bug found along the way has its own test
