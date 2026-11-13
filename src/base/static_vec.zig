//! Fixed-capacity vector. All storage is inline; it never allocates.
//!
//! Replaces dynamic arrays everywhere in the core. Capacity is a comptime constant, so
//! exceeding it is an explicit, recoverable error (`error.Full`) for data that comes from
//! the outside world, or an assertion failure when the caller guarantees the room.

const std = @import("std");
const assert = std.debug.assert;

pub fn StaticVec(comptime T: type, comptime capacity: u32) type {
    comptime assert(capacity > 0);

    return struct {
        const Self = @This();

        pub const max_len = capacity;

        items_: [capacity]T = undefined,
        len: u32 = 0,

        pub const empty: Self = .{};

        pub fn push(self: *Self, item: T) error{Full}!void {
            if (self.len == capacity) return error.Full;
            self.items_[self.len] = item;
            self.len += 1;
        }

        pub fn pushAssumeCapacity(self: *Self, item: T) void {
            assert(self.len < capacity);
            self.items_[self.len] = item;
            self.len += 1;
        }

        pub fn pop(self: *Self) ?T {
            if (self.len == 0) return null;
            self.len -= 1;
            return self.items_[self.len];
        }

        pub fn clear(self: *Self) void {
            self.len = 0;
        }

        pub fn get(self: *const Self, index: u32) T {
            assert(index < self.len);
            return self.items_[index];
        }

        pub fn getPtr(self: *Self, index: u32) *T {
            assert(index < self.len);
            return &self.items_[index];
        }

        pub fn slice(self: *const Self) []const T {
            return self.items_[0..self.len];
        }

        pub fn sliceMut(self: *Self) []T {
            return self.items_[0..self.len];
        }

        pub fn isFull(self: *const Self) bool {
            return self.len == capacity;
        }
    };
}

test "push, get, pop and capacity" {
    var v: StaticVec(u32, 3) = .empty;
    try std.testing.expectEqual(@as(u32, 0), v.len);
    try v.push(10);
    try v.push(20);
    try v.push(30);
    try std.testing.expect(v.isFull());
    try std.testing.expectError(error.Full, v.push(40));
    try std.testing.expectEqualSlices(u32, &.{ 10, 20, 30 }, v.slice());
    try std.testing.expectEqual(@as(u32, 20), v.get(1));
    try std.testing.expectEqual(@as(?u32, 30), v.pop());
    try std.testing.expect(!v.isFull());
    try v.push(99);
    try std.testing.expectEqualSlices(u32, &.{ 10, 20, 99 }, v.slice());
}

test "pop on empty and clear" {
    var v: StaticVec(f64, 2) = .empty;
    try std.testing.expectEqual(@as(?f64, null), v.pop());
    v.pushAssumeCapacity(1.5);
    v.clear();
    try std.testing.expectEqual(@as(u32, 0), v.len);
    try std.testing.expectEqual(@as(usize, 0), v.slice().len);
}

test "getPtr and sliceMut mutate in place" {
    var v: StaticVec(u8, 4) = .empty;
    try v.push(1);
    try v.push(2);
    v.getPtr(0).* = 7;
    v.sliceMut()[1] = 8;
    try std.testing.expectEqualSlices(u8, &.{ 7, 8 }, v.slice());
}
