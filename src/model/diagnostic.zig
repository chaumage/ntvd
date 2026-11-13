//! Human-readable explanation of why a model was rejected.
//!
//! Compile errors are an enum of causes (stable, testable); the diagnostic carries the
//! specifics (which components, which dimensions) for the engineer. It owns its storage so
//! it can be filled without allocating.

const std = @import("std");

pub const Diagnostic = struct {
    buf: [256]u8 = undefined,
    len: u16 = 0,

    pub fn set(self: *Diagnostic, comptime fmt: []const u8, args: anytype) void {
        const written = std.fmt.bufPrint(&self.buf, fmt, args) catch {
            // Message longer than the buffer: keep the truncated prefix.
            self.len = self.buf.len;
            return;
        };
        self.len = @intCast(written.len);
    }

    pub fn message(self: *const Diagnostic) []const u8 {
        return self.buf[0..self.len];
    }
};

test "formats a message" {
    var d: Diagnostic = .{};
    try std.testing.expectEqualStrings("", d.message());
    d.set("component '{s}' has {d} problems", .{ "wing", 2 });
    try std.testing.expectEqualStrings("component 'wing' has 2 problems", d.message());
}

test "a too-long message is truncated, never overflows" {
    var d: Diagnostic = .{};
    const long: [400]u8 = @splat('x');
    d.set("{s}", .{&long});
    try std.testing.expectEqual(@as(usize, 256), d.message().len);
}
