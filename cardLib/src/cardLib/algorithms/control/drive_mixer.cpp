// drive_mixer.cpp
// Why: the SPEC section 7 mixer as a wrapper over the tank case of drive_kinematics, and the
//      millivolt conversion.
// Owns: nothing; no state.
#include "cardLib/algorithms/control/drive_mixer.hpp"

#include <algorithm>
#include <cmath>

#include "cardLib/algorithms/control/drive_kinematics.hpp"

namespace cardLib {

SideSpeeds mixDrive(double linearSpeed, double angularSpeed) {
  const WheelSpeeds wheels =
      UNIT_TANK_KINEMATICS.wheelSpeeds({.forward = linearSpeed, .turn = angularSpeed});
  return {wheels.speeds[0], wheels.speeds[1]};
}

std::int32_t speedToMillivolts(double speed) {
  const double millivolts = std::round(speed / MAX_SPEED * MAX_MILLIVOLTS);
  return static_cast<std::int32_t>(std::clamp(millivolts, -static_cast<double>(MAX_MILLIVOLTS),
                                              static_cast<double>(MAX_MILLIVOLTS)));
}

}  // namespace cardLib
