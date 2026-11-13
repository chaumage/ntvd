# Style: TigerStyle, applied here

The philosophy comes from TigerBeetle's TigerStyle: safety first, then performance, then
developer experience; simple and explicit over clever; assertions as documentation that
executes. This file lists how it applies to this project, and which rules are **checked by a
machine** (`zig build tidy`, also part of `zig build test`) versus left to review.

## Checked by `test/tidy.zig`

| Rule | Why |
|---|---|
| Lines at most 100 columns; no tabs, trailing spaces or CRLF | one reviewable layout; `zig fmt` clean |
| Functions at most 70 lines | a function fits in one screen and one head |
| Every source file opens with a `//!` comment | each file says what it is for |
| `TODO`/`FIXME` carry an issue: `TODO(#123)` | no anonymous promises |
| No allocators or growable containers in `src/` | the core allocates nothing after startup |
| A layer imports only the layers below it; no relative imports across layers | architecture enforced, not drawn |

The checker has a self-test that proves each rule fires on a bad input. A rule that needs
judgment does not belong in it.

## Practiced, reviewed by people

**Static allocation.** Capacities live in `base/limits.zig`. Data that comes from the outside
world and can exceed a limit returns a precise error (`error.Full`, `TooManyComponents`);
a limit the caller guarantees is an assertion.

**Assertions.** Assert preconditions, postconditions and invariants, at least two per
non-trivial function, and across the boundary of every function that takes external data.
Assertions are on in Debug and `ReleaseSafe`. Run anything long in `ReleaseSafe`, never
`ReleaseFast`.

**Two kinds of failure, kept apart.**
- *Programmer error* (a violated invariant): `assert`. The process stops; it must never be
  caught.
- *Operating error* (bad user parameters, a model used outside its range, a solver that did
  not converge, a diverging simulation): an error value or a `Validity` flag. Expected and
  frequent during optimization. Never an assertion.

**Bounded everything.** No recursion. Every loop has a static upper bound, and a bound that
cannot be reached in a correct program ends in `unreachable` (see the binary search in
`base/interp.zig`). Every iterative solver takes a maximum iteration count.

**NaN is a bug, never a result.** Components return finite numbers for finite inputs and
flag the quality of the answer with `Validity`. `Plan` evaluation is total (see DESIGN D5).

**Deterministic.** No wall-clock, no global state, no iteration over unordered containers.
Randomness is an explicit `Rng` value owned by the caller.

**Explicit types and no hidden control flow.** Stored indices have explicit widths (`u16`,
`u32`), not `usize`. Dispatch is a visible `switch` over a tagged union, no vtables. Pass
large structs by pointer, and by `*const` unless mutated.

**Naming.** Units in names of quantities that are not obvious in context (`mass_kg`,
`delta_t_k`, `alpha_stall_rad`). Ports use the physical name (`airspeed`, `lift`, `drag`);
their dimension is in the signature. Inside the core everything is SI.

**Comments say why.** Every model documents its equations, its validity range and what it
returns outside it, at the top of its file. Tests use hand-calculable numbers with the
arithmetic in a comment.

**Tests.** A model is not done until it has: an analytic or hand-calculated case, a scaling or
monotonicity property where physics demands one, a validity test on both sides of its
envelope, parameter validation, and the generic contract test. Prefer deterministic seeded
loops to single examples. A bug becomes a test before it becomes a fix.

**Dependencies.** None beyond `std`. Small and auditable beats convenient.

## Zig 0.17 notes (things that cost time)

- Array repetition uses `@splat`; `**` is not accepted in this version.
- `@typeInfo(T).@"struct"` exposes `field_names`, `field_types`, `field_attrs` (not `fields`).
- `zig fmt` migrates deprecated builtins (`@enumFromInt` becomes `@fromBackingInt`); run it
  before committing.
- A struct holding slices only *borrows* them. Returning a `Spec` built from `&.{ ... }`
  containing runtime values dangles: return the backing array by value and borrow it in the
  caller (see `falling_body.components` in `model/scenarios.zig`).
- `std.testing.io` provides an `Io` for tests that touch the filesystem.
