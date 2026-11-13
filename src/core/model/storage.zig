//! Alignment-aware carving of typed slices out of a caller-owned byte buffer.
//!
//! The core never allocates. `Carver` and `Sizer` are the two halves of the exact-storage
//! contract: `Sizer` computes how many bytes a layout needs, `Carver` produces the slices from
//! that many bytes. Both must see the same sequence of `add`/`take` calls, in the same order.
//! `compileInto` and the workspace builders assert that the final cursor equals the measured
//! size, so a drift between the two is caught in tests.

const std = @import("std");
const assert = std.debug.assert;

/// Every carved type must be aligned no more strictly than this.
pub const alignment = 8;

/// A caller-owned buffer of the alignment every core plan and workspace requires.
pub const Buffer = []align(alignment) u8;

/// Accumulates the byte size of a sequence of typed arrays, including alignment padding.
pub const Sizer = struct {
    cursor: usize = 0,

    pub fn add(self: *Sizer, comptime T: type, n: usize) void {
        self.cursor = std.mem.alignForward(usize, self.cursor, @alignOf(T)) + n * @sizeOf(T);
    }

    pub fn bytes(self: Sizer) usize {
        return self.cursor;
    }
};

/// Produces typed slices from a buffer previously measured by a `Sizer`.
pub const Carver = struct {
    buffer: Buffer,
    cursor: usize = 0,

    pub fn init(buffer: Buffer) Carver {
        return .{ .buffer = buffer };
    }

    /// One value of `T`, aligned. Used for the header at the start of a workspace.
    pub fn takeOne(self: *Carver, comptime T: type) *T {
        const at = std.mem.alignForward(usize, self.cursor, @alignOf(T));
        const end = at + @sizeOf(T);
        assert(end <= self.buffer.len);
        const ptr: *T = @ptrCast(@alignCast(self.buffer.ptr + at));
        self.cursor = end;
        return ptr;
    }

    /// `n` values of `T`, aligned and contiguous.
    pub fn take(self: *Carver, comptime T: type, n: usize) []T {
        const at = std.mem.alignForward(usize, self.cursor, @alignOf(T));
        const end = at + n * @sizeOf(T);
        assert(end <= self.buffer.len);
        const ptr: [*]T = @ptrCast(@alignCast(self.buffer.ptr + at));
        self.cursor = end;
        return ptr[0..n];
    }
};

test "sizer and carver agree on a mixed layout" {
    var sizer: Sizer = .{};
    sizer.add(u8, 3);
    sizer.add(f64, 2);
    sizer.add(u16, 5);
    sizer.add(f64, 0);
    sizer.add(u8, 1);

    var buffer: [256]u8 align(alignment) = undefined;
    const size = sizer.bytes();
    try std.testing.expect(size <= buffer.len);

    var carver: Carver = .init(&buffer);
    const bytes = carver.take(u8, 3);
    const values = carver.take(f64, 2);
    const counters = carver.take(u16, 5);
    const empty = carver.take(f64, 0);
    _ = carver.take(u8, 1);

    try std.testing.expectEqual(size, carver.cursor);
    try std.testing.expectEqual(@as(usize, 3), bytes.len);
    try std.testing.expectEqual(@as(usize, 2), values.len);
    try std.testing.expectEqual(@as(usize, 5), counters.len);
    try std.testing.expectEqual(@as(usize, 0), empty.len);
}

test "takeOne aligns the header and advances past it" {
    var buffer: [64]u8 align(alignment) = undefined;
    var carver: Carver = .init(&buffer);
    const header = carver.takeOne(struct { a: f64, b: u16 });
    header.* = .{ .a = 1.5, .b = 7 };
    const tail = carver.take(u64, 1);
    tail[0] = 9;
    try std.testing.expectEqual(@as(f64, 1.5), header.a);
    try std.testing.expectEqual(@as(u16, 7), header.b);
    try std.testing.expectEqual(@as(u64, 9), tail[0]);
}
