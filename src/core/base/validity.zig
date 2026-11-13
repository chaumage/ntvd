//! Validity status of a physical evaluation.
//!
//! An optimizer will inevitably probe regions where a model does not apply. The core
//! must say so explicitly rather than return a plausible-looking number. Every model
//! returns finite outputs for any finite input, together with one of these statuses.

const std = @import("std");

pub const Validity = enum(u8) {
    /// The inputs are inside the range the model was built or calibrated for.
    valid = 0,
    /// The result is finite but was obtained by clamping or extrapolating beyond the
    /// calibrated range (stall, end of a measured table, ...). Use with caution;
    /// optimizers should penalize it.
    beyond_envelope = 1,
    /// The inputs are physically impossible (negative density, ...). Outputs are neutral
    /// finite values and carry no physical meaning.
    invalid = 2,

    /// The more severe of two statuses.
    pub fn worst(a: Validity, b: Validity) Validity {
        return @fromBackingInt(@intCast(@max(@backingInt(a), @backingInt(b))));
    }

    pub fn isValid(self: Validity) bool {
        return self == .valid;
    }
};

test "worst keeps the most severe status" {
    const all = [_]Validity{ .valid, .beyond_envelope, .invalid };
    for (all) |a| {
        try std.testing.expectEqual(a, a.worst(a));
        for (all) |b| {
            try std.testing.expectEqual(a.worst(b), b.worst(a));
            const expected: u8 = @max(@backingInt(a), @backingInt(b));
            try std.testing.expectEqual(expected, @backingInt(a.worst(b)));
        }
    }
    try std.testing.expect(Validity.valid.isValid());
    try std.testing.expect(!Validity.beyond_envelope.isValid());
}
