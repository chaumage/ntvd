# analysis (planned)

Steady analyses built on the core solver: trim points, parameter sweeps and mission
segments. This directory is reserved; it contains no code yet.

When it lands, `build.zig` will add an `analysis` module importing the core layers it needs
(`base`, `model`, `solve`), and `test/tidy.zig` will enforce that boundary. A C ABI will
eventually sit alongside it or just above it.
