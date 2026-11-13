# ntvd

**Non-terrestrial vehicle design.** An independent computation core, in Zig, for sizing and
evaluating vehicle architectures (aircraft, boats, foilers, anything built from components
with physical equations), with room around it for the tools that will use it.

An engineer describes a vehicle as **components wired together**, picks for each component a
**model** (a first-order formula today, measured data or a detailed simulation tomorrow), and
the core evaluates it: instantaneous forces, time histories, and (planned) trim points and
optimization.

Zero dependencies beyond the Zig standard library. No allocation after startup. Deterministic.

## Quick start

Requires Zig **0.17.1** (the `std` API moves between releases; the version is pinned in
`build.zig.zon`).

```sh
zig build test                  # everything: unit, property, scenario, fuzz, style
zig build tidy                  # style rules only
zig build fuzz -Dfuzz-iterations=200000 -Dfuzz-seed=42 -Dfuzz-verbose=true -Doptimize=ReleaseSafe
zig build check                 # compile without running
```

Use `-Doptimize=ReleaseSafe` for anything long-running: it keeps the assertions, which are
part of this project's safety model. `ReleaseFast` removes them, for benchmarking only.
`-Dfuzz-seed` takes a decimal number.

## What it looks like

A body falling through air, slowed by drag. The loop `velocity -> drag -> force ->
acceleration -> velocity` closes through the mass's state, so no algebraic solver is needed.
(Full version in `src/core/model/scenarios.zig`, integrated and checked against the analytic
terminal velocity in `src/core/solve/integrate.zig`.)

```zig
const spec: model.Spec = .{
    .components = &.{
        .{ .name = "body",    .component = .{ .point_mass = .{ .rigid = .{ .mass_kg = 2.0 } } } },
        .{ .name = "surface", .component = .{ .lift_surface = .{ .linear = .{ ... } } } },
        .{ .name = "air",     .component = .{ .air = .{ .isa = .{} } } },
    },
    .externals = &.{
        .{ .name = "altitude", .dim = base.dim.length },
        .{ .name = "alpha",    .dim = base.dim.angle },
        .{ .name = "weight",   .dim = base.dim.force },
    },
    .connections = &.{
        .{ .from = "altitude",      .to = "air.altitude" },
        .{ .from = "air.density",   .to = "surface.density" },
        .{ .from = "body.velocity", .to = "surface.airspeed" },
        .{ .from = "alpha",         .to = "surface.alpha" },
        .{ .from = "weight",        .to = "body.force" },
        .{ .from = "surface.drag",  .to = "body.force", .gain = -1.0 },
    },
};

var plan: model.Plan = undefined;       // 82 KB: create once, pass by pointer
var diag: model.Diagnostic = .{};
try model.compile(&spec, &plan, &diag); // or an error, with diag.message() explaining
```

To replace the analytic wing by measured wind-tunnel data, change **one value**, from
`.lift_surface = .{ .linear = ... }` to `.lift_surface = .{ .polar_table = ... }`. The
connections do not change. That is the whole mechanism for gaining fidelity over a project.

## Layout

The core solver is isolated under `src/core/`. Everything that will sit above it gets its own
top-level directory, so adding a GUI or a CLI never disturbs the solver.

| Directory | Layer | Role |
|---|---|---|
| `src/core/base` | L0 | dimensions, units, lookup tables, linear algebra, rng, atmosphere |
| `src/core/components` | L1 | the component contract and the physical models |
| `src/core/model` | L2 | architecture spec, compiler, immutable evaluation plan |
| `src/core/solve` | L3 | numerical methods acting on a plan (time integration today) |
| `src/analysis/` | planned | trim points, sweeps, mission segments |
| `src/optimize/` | planned | design-variable optimization over analyses |
| `src/io/` | planned | spec and result file formats |
| `src/cli/` | planned | command-line front end |
| `src/gui/` | planned | desktop graphical front end |
| `test/` | | architecture fuzzer, style checker |
| `docs/` | | design decisions, style rules, how to add a component |

Core layers depend only downward, and the build graph enforces it: a layer can import only the
modules it is given in `build.zig`. The same rule is reserved for the planned layers in
`test/tidy.zig`; each placeholder directory has a README describing its intended role.

## Documents

- [`docs/DESIGN.md`](docs/DESIGN.md): architecture, decisions and their rationale, roadmap
- [`docs/TIGER_STYLE.md`](docs/TIGER_STYLE.md): the coding rules and which ones a machine checks
- [`docs/ADDING_A_COMPONENT.md`](docs/ADDING_A_COMPONENT.md): add a kind or a new model

## Status

Early. The skeleton is complete and tested end to end; the physics library is small. See the
roadmap in `docs/DESIGN.md` for what exists and what is next (trim solver, mission analysis,
optimizer, file formats, more components).
