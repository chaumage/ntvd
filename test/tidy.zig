//! Style rules, enforced as a test. Run from the project root (`zig build tidy`).
//!
//! TigerStyle only stays true if a machine checks it. Every rule below is mechanical on
//! purpose; a rule that needs judgment belongs in review, not here.
//!
//!   * lines at most 100 columns, no tabs, no trailing whitespace, no CRLF
//!   * functions at most 70 lines
//!   * every source file opens with a `//!` doc comment saying what it is for
//!   * `TODO` and `FIXME` must carry an issue reference, `TODO(#123)`
//!   * no dynamic memory in the core (`src/core/`): no allocators, no growable containers
//!   * layering: a layer imports only the layers below it, and never by relative path
//!
//! A violation is reported with file and line; all violations are listed before failing.

const std = @import("std");

const max_line_columns = 100;
const max_fn_lines = 70;

/// Source directory of each layer and the module names it may import besides `std`.
///
/// The core layers under `src/core/` are built today. The directories after them are
/// reserved for layers above the core solver; the same import rules will apply as soon as
/// they contain Zig code.
const layers = [_]struct { dir: []const u8, may_import: []const []const u8 }{
    .{ .dir = "src/core/base/", .may_import = &.{} },
    .{ .dir = "src/core/components/", .may_import = &.{"base"} },
    .{ .dir = "src/core/model/", .may_import = &.{ "base", "components" } },
    .{ .dir = "src/core/solve/", .may_import = &.{ "base", "model" } },
    .{ .dir = "src/analysis/", .may_import = &.{ "base", "model", "solve" } },
    .{ .dir = "src/optimize/", .may_import = &.{ "base", "model", "solve", "analysis" } },
    .{ .dir = "src/io/", .may_import = &.{ "base", "model" } },
    .{ .dir = "src/cli/", .may_import = &.{
        "base", "model", "solve", "analysis", "optimize", "io",
    } },
    .{ .dir = "src/gui/", .may_import = &.{
        "base", "model", "solve", "analysis", "optimize", "io",
    } },
};

const banned_in_core = [_][]const u8{
    "std.heap",
    "Allocator",
    "ArrayList",
    "HashMap",
    "std.testing.allocator",
};

/// A function whose closing brace has not been seen yet.
const OpenFn = struct { line: usize, indent: usize };

const Report = struct {
    count: u32 = 0,
    /// The self-test counts violations without printing them.
    quiet: bool = false,

    fn fail(
        self: *Report,
        file: []const u8,
        line: usize,
        comptime fmt: []const u8,
        args: anytype,
    ) void {
        self.count += 1;
        if (!self.quiet) std.debug.print("{s}:{d}: " ++ fmt ++ "\n", .{ file, line } ++ args);
    }
};

test "source tree follows the style rules" {
    const gpa = std.testing.allocator;
    const io = std.testing.io;
    var report: Report = .{};

    for ([_][]const u8{ "src", "test" }) |root| {
        var dir = try std.Io.Dir.cwd().openDir(io, root, .{ .iterate = true });
        defer dir.close(io);
        var walker = try dir.walk(gpa);
        defer walker.deinit();
        while (try walker.next(io)) |entry| {
            if (entry.kind != .file or !std.mem.endsWith(u8, entry.path, ".zig")) continue;
            const full = try std.fmt.allocPrint(gpa, "{s}/{s}", .{ root, entry.path });
            defer gpa.free(full);
            const text = try dir.readFileAlloc(io, entry.path, gpa, .limited(1 << 20));
            defer gpa.free(text);
            checkFile(full, text, &report);
        }
    }
    if (report.count > 0) {
        std.debug.print("{d} style violation(s)\n", .{report.count});
        return error.StyleViolations;
    }
}

fn checkFile(file: []const u8, text: []const u8, report: *Report) void {
    const is_src = std.mem.startsWith(u8, file, "src/");
    const is_core = std.mem.startsWith(u8, file, "src/core/");
    checkHeader(file, text, report);

    var line_no: usize = 0;
    var lines = std.mem.splitScalar(u8, text, '\n');
    var fn_start: ?OpenFn = null;
    while (lines.next()) |line| {
        line_no += 1;
        checkLine(file, line_no, line, report);
        if (is_core) checkBanned(file, line_no, line, report);
        if (is_src) checkImports(file, line_no, line, report);
        checkFunctionLength(file, line_no, line, &fn_start, report);
    }
}

fn checkHeader(file: []const u8, text: []const u8, report: *Report) void {
    if (!std.mem.startsWith(u8, text, "//!")) {
        report.fail(file, 1, "file must open with a //! doc comment", .{});
    }
}

fn checkLine(file: []const u8, n: usize, line: []const u8, report: *Report) void {
    if (std.mem.indexOfScalar(u8, line, '\t') != null) report.fail(file, n, "tab character", .{});
    if (std.mem.endsWith(u8, line, "\r")) report.fail(file, n, "CRLF line ending", .{});
    const trimmed = std.mem.trimEnd(u8, line, "\r");
    if (trimmed.len != std.mem.trimEnd(u8, trimmed, " ").len) {
        report.fail(file, n, "trailing whitespace", .{});
    }
    const columns = std.unicode.utf8CountCodepoints(trimmed) catch trimmed.len;
    if (columns > max_line_columns) {
        report.fail(file, n, "line is {d} columns, limit is {d}", .{ columns, max_line_columns });
    }
    // This file describes the rule, so it necessarily spells out the words.
    if (std.mem.eql(u8, file, "test/tidy.zig")) return;
    for ([_][]const u8{ "TODO", "FIXME" }) |tag| {
        const at = std.mem.indexOf(u8, line, tag) orelse continue;
        const after = line[at + tag.len ..];
        if (!std.mem.startsWith(u8, after, "(#")) {
            report.fail(file, n, "{s} without an issue reference, write {s}(#123)", .{ tag, tag });
        }
    }
}

fn checkBanned(file: []const u8, n: usize, line: []const u8, report: *Report) void {
    if (isComment(line)) return;
    for (banned_in_core) |word| {
        if (std.mem.indexOf(u8, line, word) != null) {
            report.fail(
                file,
                n,
                "'{s}' is banned in src/core/: the core allocates nothing",
                .{word},
            );
        }
    }
}

fn checkImports(file: []const u8, n: usize, line: []const u8, report: *Report) void {
    if (isComment(line)) return;
    const marker = "@import(\"";
    const at = std.mem.indexOf(u8, line, marker) orelse return;
    const rest = line[at + marker.len ..];
    const end = std.mem.indexOfScalar(u8, rest, '"') orelse return;
    const target = rest[0..end];

    if (std.mem.startsWith(u8, target, "..")) {
        report.fail(file, n, "relative import '{s}' crosses a layer boundary", .{target});
        return;
    }
    if (std.mem.endsWith(u8, target, ".zig") or std.mem.eql(u8, target, "std")) return;
    if (std.mem.eql(u8, target, "builtin")) return;

    for (layers) |layer| {
        if (!std.mem.startsWith(u8, file, layer.dir)) continue;
        for (layer.may_import) |allowed| {
            if (std.mem.eql(u8, allowed, target)) return;
        }
        report.fail(file, n, "layer {s} may not import '{s}'", .{ layer.dir, target });
    }
}

fn isComment(line: []const u8) bool {
    return std.mem.startsWith(u8, std.mem.trimStart(u8, line, " "), "//");
}

/// Tracks `fn` declarations by indentation: a function ends at the first later line that
/// is a lone closing brace at the same indentation.
fn checkFunctionLength(
    file: []const u8,
    n: usize,
    line: []const u8,
    open: *?OpenFn,
    report: *Report,
) void {
    const indent = line.len - std.mem.trimStart(u8, line, " ").len;
    const body = line[indent..];
    if (open.*) |start| {
        if (indent == start.indent and std.mem.eql(u8, std.mem.trimEnd(u8, body, "\r"), "}")) {
            const length = n - start.line + 1;
            if (length > max_fn_lines) {
                report.fail(file, start.line, "function is {d} lines, limit is {d}", .{
                    length, max_fn_lines,
                });
            }
            open.* = null;
        }
        return;
    }
    const declares = std.mem.startsWith(u8, body, "fn ") or std.mem.startsWith(u8, body, "pub fn ");
    if (declares and std.mem.endsWith(u8, std.mem.trimEnd(u8, body, "\r"), "{")) {
        open.* = .{ .line = n, .indent = indent };
    }
}

test "the checker itself flags what it should" {
    var report: Report = .{ .quiet = true };
    var long_line: [140]u8 = undefined;
    const prefix = "//! doc\nconst x = 1; // ";
    @memcpy(long_line[0..prefix.len], prefix);
    @memset(long_line[prefix.len..], 'x');
    checkFile("src/core/base/fake.zig", &long_line, &report);
    try std.testing.expectEqual(@as(u32, 1), report.count);

    report = .{ .quiet = true };
    checkFile("src/core/base/fake.zig", "const a = 1;\n", &report);
    try std.testing.expectEqual(@as(u32, 1), report.count); // missing header

    report = .{ .quiet = true };
    checkFile("src/core/base/fake.zig", "//! d\nconst m = @import(\"model\");\n", &report);
    try std.testing.expectEqual(@as(u32, 1), report.count); // base may not import model

    report = .{ .quiet = true };
    const imports = "//! d\nconst b = @import(\"base\");\nconst z = @import(\"../base/x.zig\");\n";
    checkFile("src/core/model/fake.zig", imports, &report);
    try std.testing.expectEqual(@as(u32, 1), report.count); // relative import

    report = .{ .quiet = true };
    checkFile("src/core/base/fake.zig", "//! d\n// TODO fix\nvar a: std.heap.X = 1;\n", &report);
    try std.testing.expectEqual(@as(u32, 2), report.count); // bare TODO, banned heap
}
