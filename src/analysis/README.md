# analysis

Steady analyses built on the compiled model and the numerical solvers. This is layer L4: it
imports `base`, `model` and `solve`, and `build.zig` creates the corresponding Zig module.

## Today

- `trim.zig` finds steady-flight points. The first version solves the reduced, symmetric
  level-flight trim of a two-wing aircraft at fixed speed: the unknowns are `alpha` (bound to
  the body velocity and attitude state slots) and `throttle` (a plan external); the residuals
  are the body x and z force derivatives. Side force and moments are verified zero by symmetry.

- Newton iteration lives one layer down, in `src/core/solve/newton.zig`.

## Next

Envelope sweeps (maximum speed in a direction, grid scans with limiting-constraint reports),
then mission segments. A C ABI will eventually sit alongside or just above this layer.
