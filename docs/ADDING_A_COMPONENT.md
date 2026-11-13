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
       pub fn outputs(self: *const SweptLinear, x: Model.State, u: Model.In)
           contract.Evaluated(Model.Out) { ... }
   };
   ```
2. Add it as a variant of the kind's `Model` union, and a branch in `validate`/`outputs`
   (the `inline else` already covers it).
3. Document at the top of the file: the equations, the validity range, and what is returned
   outside it.
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
           .in_dims = .{ .speed = base.dim.speed, .density = base.dim.density },
           .out_dims = .{ .drag = base.dim.force },
           .feedthrough = true,              // must be true when there are no states
       });
       pub fn validate(...) ...
       pub fn outputs(...) ...
       // pub fn derivatives(...)  only if State has fields
   };
   ```
   Every field of `In`, `Out` and `State` must be `f64` and must appear in the dimension
   lists; the compiler tells you precisely what is missing. Mark inputs that add up (forces)
   with `.in_sum = .{ .force = true }`.
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

## Checklist before merging

- [ ] `zig build test` passes in Debug and with `-Doptimize=ReleaseSafe`
- [ ] `zig fmt --check build.zig src test` is clean
- [ ] file header documents equations, validity range, behavior outside it
- [ ] no allocation, no recursion, every loop bounded
- [ ] a bug found along the way has its own test
