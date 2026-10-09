// wheel_distance.cpp
// Why: implements the SPEC section 5 step 1 unit conversions.
// Owns: nothing; no state.
#include "cardLib/algorithms/localization/wheel_distance.hpp"

#include <numbers>

namespace cardLib {

namespace {
constexpr double CENTIDEGREES_PER_TURN = 36000.0;
constexpr double DEGREES_PER_TURN = 360.0;
constexpr double ADI_TICKS_PER_TURN = 360.0;
constexpr double SECONDS_PER_MINUTE = 60.0;
}  // namespace

double rotationSensorInches(double centidegrees, double wheelDiameterInches, double gearRatio) {
  return centidegrees / CENTIDEGREES_PER_TURN * std::numbers::pi * wheelDiameterInches * gearRatio;
}

double motorEncoderInches(double degrees, double wheelDiameterInches, double externalRatio,
                          double driveScale) {
  return degrees / DEGREES_PER_TURN * std::numbers::pi * wheelDiameterInches * externalRatio *
         driveScale;
}

double encoderDegreesToInches(double degrees, double wheelDiameterInches, double gearRatio) {
  return degrees / DEGREES_PER_TURN * std::numbers::pi * wheelDiameterInches * gearRatio;
}

double adiEncoderInches(double ticks, double wheelDiameterInches, double gearRatio) {
  return ticks / ADI_TICKS_PER_TURN * std::numbers::pi * wheelDiameterInches * gearRatio;
}

double maxWheelSpeedInchesPerSecond(double wheelDiameterInches, double wheelRpm) {
  return std::numbers::pi * wheelDiameterInches * wheelRpm / SECONDS_PER_MINUTE;
}

}  // namespace cardLib
