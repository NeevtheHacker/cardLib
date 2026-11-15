// field_centric.cpp
// Why: implements the SPEC 20.11 stick rotation with the one field-to-robot formula.
// Owns: nothing; no state.
#include "cardLib/algorithms/control/field_centric.hpp"

#include <algorithm>
#include <cmath>

#include "cardLib/algorithms/control/drive_command.hpp"

#include "cardLib/algorithms/math/pose.hpp"

namespace cardLib {

StickVector fieldCentricSticks(double forwardStick, double strafeStick, double headingDegrees) {
  const LocalOffset robot = fieldToLocal({0.0, 0.0, headingDegrees}, {strafeStick, forwardStick});
  return {robot.forwardInches, robot.rightInches};
}

StickVector curvedFieldCentricSticks(double forwardStick, double strafeStick,
                                     double headingDegrees, const DriveCurve& curve) {
  const double length = std::hypot(forwardStick, strafeStick);
  if (length == 0.0) {
    return {};
  }
  const double scale = applyDriveCurve(std::min(length, MAX_SPEED), curve) / length;
  return fieldCentricSticks(forwardStick * scale, strafeStick * scale, headingDegrees);
}

}  // namespace cardLib
