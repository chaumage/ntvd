//! Conversion factors between common engineering units and SI.
//!
//! The core is SI-only. Conversion happens at the boundary (file input, user interface) by
//! multiplying with these factors: `speed_ms = speed_knots * units.knot`.
//! Every factor is exact by definition except where noted.

const std = @import("std");

pub const pi: f64 = std.math.pi;

/// Standard gravity, m/s^2 (exact, by definition).
pub const g0: f64 = 9.80665;

// Angles
pub const degree: f64 = pi / 180.0;

// Length (to m)
pub const inch: f64 = 0.0254;
pub const foot: f64 = 0.3048;
pub const kilometer: f64 = 1000.0;
pub const nautical_mile: f64 = 1852.0;

// Time (to s)
pub const minute: f64 = 60.0;
pub const hour: f64 = 3600.0;

// Speed (to m/s)
pub const knot: f64 = nautical_mile / hour;
pub const kilometer_per_hour: f64 = kilometer / hour;

// Mass (to kg) and force (to N)
pub const pound_mass: f64 = 0.45359237;
pub const pound_force: f64 = pound_mass * g0;
pub const kilogram_force: f64 = g0;

// Power (to W)
pub const horsepower_mechanical: f64 = 550.0 * foot * pound_force;
pub const kilowatt: f64 = 1000.0;

// Energy (to J)
pub const watt_hour: f64 = 3600.0;
pub const kilowatt_hour: f64 = 3.6e6;

test "exact relations between units" {
    try std.testing.expectApproxEqRel(@as(f64, 0.514444444444), knot, 1e-12);
    try std.testing.expectApproxEqRel(foot / inch, @as(f64, 12.0), 1e-14);
    try std.testing.expectApproxEqRel(pound_force, @as(f64, 4.4482216152605), 1e-12);
    try std.testing.expectApproxEqRel(kilowatt_hour, 1000.0 * watt_hour, 1e-14);
}

test "horsepower and degree against reference values" {
    // 1 hp (mechanical) = 745.6998715822702 W
    try std.testing.expectApproxEqRel(@as(f64, 745.6998715822702), horsepower_mechanical, 1e-12);
    try std.testing.expectApproxEqAbs(pi, 180.0 * degree, 1e-15);
    // Round trip: 90 knots -> m/s -> knots.
    const v = 90.0 * knot;
    try std.testing.expectApproxEqRel(@as(f64, 90.0), v / knot, 1e-14);
}
