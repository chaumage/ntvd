# Core solver

The ntvd computation core: components, architecture compilation, evaluation plans and
numerical methods. This is the part that exists today.

| Directory | Layer | Role |
|---|---|---|
| `base/` | L0 | dimensions, units, validity, tables, linear algebra, rng, atmosphere |
| `components/` | L1 | the component contract and physical models |
| `model/` | L2 | architecture spec, compiler, immutable evaluation plan |
| `solve/` | L3 | time integration; Newton and trim later |

Dependencies point downward only. `build.zig` creates one module per layer and gives it only
the lower layers it may import. `test/tidy.zig` repeats the rule on import statements and
enforces that nothing under `src/core/` allocates or uses growable containers.

Start with [`docs/DESIGN.md`](../../docs/DESIGN.md) for the architecture and
[`docs/ADDING_A_COMPONENT.md`](../../docs/ADDING_A_COMPONENT.md) to extend the physics.
