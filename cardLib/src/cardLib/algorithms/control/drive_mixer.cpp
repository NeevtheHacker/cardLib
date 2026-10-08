// drive_mixer.cpp
// Why: implements the SPEC section 7 mixer, desaturation and millivolt conversion.
// Owns: nothing; no state.
#include "cardLib/algorithms/control/drive_mixer.hpp"

#include <algorithm>
#include <cmath>

namespace cardLib {

SideSpeeds mixDrive(double linearSpeed, double angularSpeed) {
  const double angular = std::clamp(angularSpeed, -MAX_SPEED, MAX_SPEED);
  double linear = linearSpeed;
  if (std::abs(linear) + std::abs(angular) > MAX_SPEED) {
    linear = std::copysign(MAX_SPEED - std::abs(angular), linear);
  }
  return {linear + angular, linear - angular};
}

std::int32_t speedToMillivolts(double speed) {
  const double millivolts = std::round(speed / MAX_SPEED * MAX_MILLIVOLTS);
  return static_cast<std::int32_t>(std::clamp(millivolts, -static_cast<double>(MAX_MILLIVOLTS),
                                              static_cast<double>(MAX_MILLIVOLTS)));
}

}  // namespace cardLib
