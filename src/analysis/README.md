# analysis

Steady analyses built on the compiled model and the numerical solvers. This is layer L4: it
imports `base`, `model` and `solve`, and `build.zig` creates the corresponding Zig module.

## Today

- `trim.zig` finds steady-flight points. It solves the reduced, symmetric case: `alpha` (bound
  to the body velocity and attitude state slots) plus one control external at fixed speed, or
  `alpha` plus speed for an unpowered glider. Flight directions are world NED vectors; the
  residuals are the body x and z force derivatives. Side force and moments are verified zero by
  symmetry.

- `envelope.zig` maximizes speed along a world direction. For a powered plan it scans speed,
  trims at each point, brackets the last feasible trim, refines and verifies it; for a glider the
  trim itself solves alpha and speed. `sweep` fills a caller-provided array with deterministic,
  ordered results. Limits are classified as thrust, lift, speed bounds, validity or none.

- Newton iteration lives one layer down, in `src/core/solve/newton.zig`.

## Next

Mission segments and design studies composed from trim and envelope points. A C ABI will
eventually sit alongside or just above this layer.
