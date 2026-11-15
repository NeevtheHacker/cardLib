// field_centric.cpp
// Why: implements the SPEC 20.11 stick rotation with the one field-to-robot formula.
// Owns: nothing; no state.
#include "cardLib/algorithms/control/field_centric.hpp"

#include "cardLib/algorithms/math/pose.hpp"

namespace cardLib {

StickVector fieldCentricSticks(double forwardStick, double strafeStick, double headingDegrees) {
  const LocalOffset robot = fieldToLocal({0.0, 0.0, headingDegrees}, {strafeStick, forwardStick});
  return {robot.forwardInches, robot.rightInches};
}

}  // namespace cardLib
