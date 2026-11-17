//! Build graph of ntvd.
//!
//! The core solver lives under `src/core/`. Layering is enforced by the compiler, not by
//! convention. Each layer is a Zig module whose root file lives in its own directory, and
//! each module only receives the lower layers listed below. A layer that tries to `@import`
//! a higher one, or to reach into another layer's directory by relative path, fails to
//! compile.
//!
//!   base        L0   dimensions, tables, linear algebra, rng           (std only)
//!   components  L1   component contract and physical models            (base)
//!   model       L2   architecture spec -> validated, compiled plan     (base, components)
//!   solve       L3   time integration, damped Newton                   (base, model)
//!   analysis    L4   steady-flight trim                                (base, model, solve)
//!
//! The directories `src/{optimize,io,cli,gui}/` are reserved for layers that will sit above
//! the core solver. They are not built yet; see their READMEs for the plan.
//!
//! Steps
//!   zig build test    all unit, property, fuzz and tidy tests
//!   zig build check   compile everything without running (fast editor feedback)
//!   zig build tidy    source style rules only
//!   zig build fuzz    only the architecture fuzzer (-Dfuzz-iterations=N -Dfuzz-seed=S)
//!                     add -Dfuzz-verbose=true to print its seed and compiled count

const std = @import("std");

const Import = std.Build.Module.Import;

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    // Debug by default for development. Assertions are part of this codebase's safety
    // model, so anything that runs for real (long fuzz runs, optimizer runs, a shipped
    // library) should use -Doptimize=ReleaseSafe, which keeps them. ReleaseFast removes
    // them and is for benchmarking only.
    const optimize = b.standardOptimizeOption(.{});
    const filters = b.option([]const []const u8, "test-filter", "Only run tests matching") orelse &.{};
    const fuzz_iterations = b.option(u32, "fuzz-iterations", "Random architectures per fuzz run") orelse 20_000;
    const fuzz_seed = b.option(u64, "fuzz-seed", "Seed of the architecture fuzzer") orelse 0x5eed;
    // A test that writes to stderr makes the Zig 0.17 build runner print the step as if it
    // had failed, and append a stale "failed command:" line. Keep the fuzzer quiet by
    // default; opt in when you want its summary.
    const fuzz_verbose = b.option(bool, "fuzz-verbose", "Print the fuzz summary") orelse false;

    const base = addLayer(b, "base", "src/core/base/root.zig", target, optimize, &.{});
    const components = addLayer(b, "components", "src/core/components/root.zig", target, optimize, &.{
        .{ .name = "base", .module = base },
    });
    const model = addLayer(b, "model", "src/core/model/root.zig", target, optimize, &.{
        .{ .name = "base", .module = base },
        .{ .name = "components", .module = components },
    });
    const solve = addLayer(b, "solve", "src/core/solve/root.zig", target, optimize, &.{
        .{ .name = "base", .module = base },
        .{ .name = "model", .module = model },
    });
    const analysis = addLayer(b, "analysis", "src/analysis/root.zig", target, optimize, &.{
        .{ .name = "base", .module = base },
        .{ .name = "model", .module = model },
        .{ .name = "solve", .module = solve },
    });

    const test_step = b.step("test", "Run all tests");
    const check_step = b.step("check", "Compile everything without running");
    const tidy_step = b.step("tidy", "Check source style rules");
    const fuzz_step = b.step("fuzz", "Run the architecture fuzzer");

    // Per-layer unit tests.
    const layers = [_]*std.Build.Module{ base, components, model, solve, analysis };
    for (layers) |layer| {
        const unit = b.addTest(.{ .root_module = layer, .filters = filters });
        check_step.dependOn(&unit.step);
        test_step.dependOn(&b.addRunArtifact(unit).step);
    }

    // Style rules, run against the source tree from the project root.
    const tidy = b.addTest(.{ .root_module = b.createModule(.{
        .root_source_file = b.path("test/tidy.zig"),
        .target = target,
        .optimize = optimize,
    }) });
    const tidy_run = b.addRunArtifact(tidy);
    tidy_run.setCwd(b.path("."));
    check_step.dependOn(&tidy.step);
    tidy_step.dependOn(&tidy_run.step);
    test_step.dependOn(&tidy_run.step);

    // Architecture fuzzer. It runs with the other tests, and alone via `zig build fuzz`
    // with a larger budget or another seed.
    const fuzz_options = b.addOptions();
    fuzz_options.addOption(u32, "iterations", fuzz_iterations);
    fuzz_options.addOption(u64, "seed", fuzz_seed);
    fuzz_options.addOption(bool, "verbose", fuzz_verbose);
    const fuzz = b.addTest(.{ .root_module = b.createModule(.{
        .root_source_file = b.path("test/fuzz_model.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "base", .module = base },
            .{ .name = "model", .module = model },
            .{ .name = "fuzz_options", .module = fuzz_options.createModule() },
        },
    }) });
    const fuzz_run = b.addRunArtifact(fuzz);
    check_step.dependOn(&fuzz.step);
    fuzz_step.dependOn(&fuzz_run.step);
    test_step.dependOn(&fuzz_run.step);
}

fn addLayer(
    b: *std.Build,
    name: []const u8,
    root: []const u8,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    imports: []const Import,
) *std.Build.Module {
    return b.addModule(name, .{
        .root_source_file = b.path(root),
        .target = target,
        .optimize = optimize,
        .imports = imports,
    });
}
