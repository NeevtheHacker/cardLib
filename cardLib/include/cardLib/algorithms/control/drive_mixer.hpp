// drive_mixer.hpp
// Why: turns linear and angular speeds into side speeds without losing the turn (SPEC 7).
// Owns: the mix, the desaturation and the speed-to-millivolt conversion.
#pragma once

#include <cstdint>

namespace cardLib {

constexpr double MAX_SPEED = 127.0;
constexpr std::int32_t MAX_MILLIVOLTS = 12000;

// Speeds on the -127 to 127 scale.
struct SideSpeeds {
  double left = 0.0;
  double right = 0.0;
};

// L = lin + ang, R = lin - ang, after clamping |ang| to 127 and shrinking lin so
// |lin| + |ang| <= 127. Positive ang turns clockwise.
SideSpeeds mixDrive(double linearSpeed, double angularSpeed);

// round(speed / 127 * 12000), clamped to +-12000. The last step before the motors.
std::int32_t speedToMillivolts(double speed);

}  // namespace cardLib
