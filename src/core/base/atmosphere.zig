//! Reference fluid properties: the International Standard Atmosphere and water constants.
//!
//! ISA covers the troposphere and the lower stratosphere (-500 m to 20 000 m). Altitude is
//! treated as geopotential altitude; the difference to geometric altitude is below 0.4 %
//! of height at the top of the range, far below the uncertainty of any sizing model.
//! Outside the range the properties are clamped to the nearest edge and flagged.

const std = @import("std");
const assert = std.debug.assert;
const Validity = @import("validity.zig").Validity;

pub const gas_constant_air: f64 = 287.05287; // J/(kg K)
pub const heat_capacity_ratio: f64 = 1.4;
pub const sea_level_temperature: f64 = 288.15; // K
pub const sea_level_pressure: f64 = 101_325.0; // Pa
pub const gravity: f64 = 9.80665; // m/s^2
pub const lapse_rate: f64 = 0.0065; // K/m, troposphere

pub const tropopause_altitude: f64 = 11_000.0; // m
pub const min_altitude: f64 = -500.0; // m
pub const max_altitude: f64 = 20_000.0; // m

const tropopause_temperature: f64 = sea_level_temperature - lapse_rate * tropopause_altitude;
const pressure_exponent: f64 = gravity / (gas_constant_air * lapse_rate);

pub const Air = struct {
    temperature: f64, // K
    pressure: f64, // Pa
    density: f64, // kg/m^3
    speed_of_sound: f64, // m/s
    dynamic_viscosity: f64, // Pa s
};

pub const AirResult = struct {
    air: Air,
    validity: Validity,
};

/// Standard atmosphere at `altitude` metres, with an optional temperature offset
/// `delta_t` (ISA+delta_t). The pressure profile stays the standard one, as in the usual
/// non-standard-day convention; the offset only changes temperature and thus density.
pub fn isa(altitude: f64, delta_t: f64) AirResult {
    assert(std.math.isFinite(altitude) and std.math.isFinite(delta_t));
    const h = std.math.clamp(altitude, min_altitude, max_altitude);
    const validity: Validity = if (h == altitude) .valid else .beyond_envelope;

    var t_std: f64 = undefined;
    var p: f64 = undefined;
    if (h <= tropopause_altitude) {
        t_std = sea_level_temperature - lapse_rate * h;
        const ratio = t_std / sea_level_temperature;
        p = sea_level_pressure * std.math.pow(f64, ratio, pressure_exponent);
    } else {
        t_std = tropopause_temperature;
        const p11 = sea_level_pressure *
            std.math.pow(f64, tropopause_temperature / sea_level_temperature, pressure_exponent);
        p = p11 * @exp(-gravity * (h - tropopause_altitude) / (gas_constant_air * t_std));
    }

    const t = t_std + delta_t;
    assert(t > 0.0); // callers validate the offset; ISA - 100 K would be absurd
    return .{ .air = .{
        .temperature = t,
        .pressure = p,
        .density = p / (gas_constant_air * t),
        .speed_of_sound = @sqrt(heat_capacity_ratio * gas_constant_air * t),
        .dynamic_viscosity = sutherland(t),
    }, .validity = validity };
}

/// Sutherland's law for the dynamic viscosity of air.
pub fn sutherland(temperature: f64) f64 {
    assert(temperature > 0.0);
    return 1.458e-6 * std.math.pow(f64, temperature, 1.5) / (temperature + 110.4);
}

/// Reference water properties (not a model: fixed values at stated conditions).
pub const water = struct {
    /// Fresh water at 20 C.
    pub const density_fresh: f64 = 998.2; // kg/m^3
    pub const kinematic_viscosity_fresh: f64 = 1.004e-6; // m^2/s
    /// Sea water, 35 g/kg salinity, at 15 C.
    pub const density_sea: f64 = 1025.0; // kg/m^3
    pub const kinematic_viscosity_sea: f64 = 1.19e-6; // m^2/s
};

test "sea level matches the standard atmosphere" {
    const r = isa(0.0, 0.0);
    try std.testing.expectEqual(Validity.valid, r.validity);
    try std.testing.expectApproxEqAbs(@as(f64, 288.15), r.air.temperature, 1e-12);
    try std.testing.expectApproxEqRel(@as(f64, 101_325.0), r.air.pressure, 1e-12);
    try std.testing.expectApproxEqRel(@as(f64, 1.2250), r.air.density, 1e-4);
    try std.testing.expectApproxEqRel(@as(f64, 340.294), r.air.speed_of_sound, 1e-5);
    try std.testing.expectApproxEqRel(@as(f64, 1.7894e-5), r.air.dynamic_viscosity, 1e-3);
}

test "tropopause matches ISA tables" {
    const r = isa(11_000.0, 0.0);
    try std.testing.expectApproxEqAbs(@as(f64, 216.65), r.air.temperature, 1e-9);
    try std.testing.expectApproxEqRel(@as(f64, 22_632.1), r.air.pressure, 1e-4);
    try std.testing.expectApproxEqRel(@as(f64, 0.36392), r.air.density, 5e-4);
}

test "20 km matches ISA tables" {
    const r = isa(20_000.0, 0.0);
    try std.testing.expectApproxEqAbs(@as(f64, 216.65), r.air.temperature, 1e-9);
    try std.testing.expectApproxEqRel(@as(f64, 5_474.89), r.air.pressure, 1e-3);
    try std.testing.expectApproxEqRel(@as(f64, 0.08803), r.air.density, 1e-3);
}

test "density and pressure decrease monotonically with altitude" {
    var prev = isa(min_altitude, 0.0).air;
    var h: f64 = min_altitude + 50.0;
    while (h <= max_altitude) : (h += 50.0) {
        const cur = isa(h, 0.0).air;
        try std.testing.expect(cur.density < prev.density);
        try std.testing.expect(cur.pressure < prev.pressure);
        prev = cur;
    }
}

test "the model is continuous across the tropopause" {
    // A 1 micrometre gap: the true pressure gradient (rho * g, about 3.6 Pa/m) contributes
    // only ~1e-10 relative, so any real jump in the profile would exceed this tolerance.
    const below = isa(10_999.999999, 0.0).air;
    const above = isa(11_000.000001, 0.0).air;
    try std.testing.expectApproxEqRel(below.pressure, above.pressure, 1e-8);
    try std.testing.expectApproxEqRel(below.density, above.density, 1e-8);
    try std.testing.expectApproxEqRel(below.temperature, above.temperature, 1e-8);
}

test "temperature offset changes density, not pressure" {
    const std_day = isa(3000.0, 0.0).air;
    const hot_day = isa(3000.0, 15.0).air;
    try std.testing.expectEqual(std_day.pressure, hot_day.pressure);
    try std.testing.expect(hot_day.density < std_day.density);
    try std.testing.expectApproxEqRel(
        std_day.density * std_day.temperature / hot_day.temperature,
        hot_day.density,
        1e-12,
    );
}

test "out of range altitude is clamped and flagged" {
    const high = isa(50_000.0, 0.0);
    try std.testing.expectEqual(Validity.beyond_envelope, high.validity);
    try std.testing.expectEqual(isa(max_altitude, 0.0).air.density, high.air.density);
    const low = isa(-5_000.0, 0.0);
    try std.testing.expectEqual(Validity.beyond_envelope, low.validity);
    try std.testing.expectEqual(isa(min_altitude, 0.0).air.density, low.air.density);
}

test "water reference values are physically ordered" {
    try std.testing.expect(water.density_sea > water.density_fresh);
    try std.testing.expect(water.kinematic_viscosity_sea > water.kinematic_viscosity_fresh);
}
