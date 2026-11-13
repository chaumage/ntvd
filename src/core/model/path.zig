//! Parsing of signal paths: "name" or "component.port".
//!
//! Names cannot contain '.', so a path has at most one separator and the split is
//! unambiguous.

const std = @import("std");

pub const Path = struct {
    /// An external signal, or the component of a "component.port" path.
    head: []const u8,
    /// The port of a "component.port" path, null for a bare external name.
    port: ?[]const u8,
};

pub const Error = error{BadPath};

pub fn parse(text: []const u8) Error!Path {
    if (text.len == 0) return error.BadPath;
    const dot = std.mem.indexOfScalar(u8, text, '.') orelse return .{ .head = text, .port = null };
    const head = text[0..dot];
    const port = text[dot + 1 ..];
    if (head.len == 0 or port.len == 0) return error.BadPath;
    if (std.mem.indexOfScalar(u8, port, '.') != null) return error.BadPath;
    return .{ .head = head, .port = port };
}

test "bare names and component ports" {
    const bare = try parse("altitude");
    try std.testing.expectEqualStrings("altitude", bare.head);
    try std.testing.expectEqual(@as(?[]const u8, null), bare.port);

    const full = try parse("wing.lift");
    try std.testing.expectEqualStrings("wing", full.head);
    try std.testing.expectEqualStrings("lift", full.port.?);
}

test "malformed paths are rejected" {
    for ([_][]const u8{ "", ".", "a.", ".b", "a.b.c", ".." }) |text| {
        try std.testing.expectError(error.BadPath, parse(text));
    }
}
