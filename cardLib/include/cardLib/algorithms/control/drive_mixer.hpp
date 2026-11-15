// drive_mixer.hpp
// Why: turns linear and angular speeds into side speeds without losing the turn (SPEC 7). Since
//      SPEC 20 it is the tank case of drive_kinematics, kept for tank code and driver control.
// Owns: the two-side mix and the speed-to-millivolt conversion.
#pragma once

#include <cstdint>

#include "cardLib/algorithms/control/drive_command.hpp"

namespace cardLib {

// Speeds on the -127 to 127 scale.
struct SideSpeeds {
  double left = 0.0;
  double right = 0.0;
};

// L = lin + ang, R = lin - ang, after clamping |ang| to 127 and shrinking lin so
// |lin| + |ang| <= 127. Positive ang turns clockwise. The same numbers as
// UNIT_TANK_KINEMATICS.wheelSpeeds({lin, 0, ang}), which is how it is computed.
SideSpeeds mixDrive(double linearSpeed, double angularSpeed);

// round(speed / 127 * 12000), clamped to +-12000. The last step before the motors.
std::int32_t speedToMillivolts(double speed);

}  // namespace cardLib
