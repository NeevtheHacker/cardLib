// wheel_distance.hpp
// Why: turns raw sensor units into inches rolled, so the cartridge can't be applied twice (SPEC 5).
// Owns: the SPEC section 5 step 1 conversions. No state.
#pragma once

namespace cardLib {

// Rotation sensor: d = c / 36000 * pi * D * g. g = wheel turns per sensor turn.
double rotationSensorInches(double centidegrees, double wheelDiameterInches, double gearRatio);

// Drive motor encoder (degrees of the output shaft, after the cartridge):
// d = deg / 360 * pi * D * G_ext * s_drive. G_ext = wheel turns per motor output turn from
// external gears only.
double motorEncoderInches(double degrees, double wheelDiameterInches, double externalRatio,
                          double driveScale);

// ADI (3-wire) encoder, 360 ticks per turn.
double adiEncoderInches(double ticks, double wheelDiameterInches, double gearRatio);

// v_max = pi * D * rpm / 60, the fastest a wheel of this size can move the chassis.
double maxWheelSpeedInchesPerSecond(double wheelDiameterInches, double wheelRpm);

}  // namespace cardLib
