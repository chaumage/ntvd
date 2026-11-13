//! Fixed-size identifier for components and external signals.
//!
//! A compiled plan must own its names (it outlives the spec that produced it) without
//! allocating, so names are copied into an inline buffer. Only letters, digits, '_' and
//! '-' are accepted; '.' is reserved as the separator of "component.port" paths.

const std = @import("std");
const assert = std.debug.assert;
const limits = @import("limits.zig");

pub const Name = struct {
    buf: [limits.max_name_len]u8 = @splat(0),
    len: u8 = 0,

    pub const Error = error{ Empty, TooLong, InvalidCharacter };

    pub fn init(text: []const u8) Error!Name {
        if (text.len == 0) return error.Empty;
        if (text.len > limits.max_name_len) return error.TooLong;
        for (text) |c| {
            if (!isNameChar(c)) return error.InvalidCharacter;
        }
        var result: Name = .{};
        @memcpy(result.buf[0..text.len], text);
        result.len = @intCast(text.len);
        return result;
    }

    pub fn slice(self: *const Name) []const u8 {
        assert(self.len > 0);
        return self.buf[0..self.len];
    }

    pub fn eql(self: *const Name, text: []const u8) bool {
        return std.mem.eql(u8, self.slice(), text);
    }
};

fn isNameChar(c: u8) bool {
    return std.ascii.isAlphanumeric(c) or c == '_' or c == '-';
}

test "valid names round-trip" {
    const n = try Name.init("wing_main-2");
    try std.testing.expectEqualStrings("wing_main-2", n.slice());
    try std.testing.expect(n.eql("wing_main-2"));
    try std.testing.expect(!n.eql("wing_main"));
}

test "invalid names are rejected" {
    try std.testing.expectError(error.Empty, Name.init(""));
    try std.testing.expectError(error.InvalidCharacter, Name.init("a.b"));
    try std.testing.expectError(error.InvalidCharacter, Name.init("has space"));
    try std.testing.expectError(error.InvalidCharacter, Name.init("naïve"));
    const too_long: [limits.max_name_len + 1]u8 = @splat('x');
    try std.testing.expectError(error.TooLong, Name.init(&too_long));
    const longest: [limits.max_name_len]u8 = @splat('x');
    const ok = try Name.init(&longest);
    try std.testing.expectEqual(@as(u8, limits.max_name_len), ok.len);
}
