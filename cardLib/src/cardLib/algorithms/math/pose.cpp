// pose.cpp
// Why: implements the SPEC section 1 robot-local <-> field transforms.
// Owns: nothing beyond the math; no state.
#include "cardLib/algorithms/math/pose.hpp"

#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

Point localToField(const Pose& pose, const LocalOffset& local) {
  const double headingRadians = degreesToRadians(pose.headingDegrees);
  const double sinHeading = std::sin(headingRadians);
  const double cosHeading = std::cos(headingRadians);
  return {pose.xInches + cosHeading * local.rightInches + sinHeading * local.forwardInches,
          pose.yInches - sinHeading * local.rightInches + cosHeading * local.forwardInches};
}

LocalOffset fieldToLocal(const Pose& pose, const Point& point) {
  const double headingRadians = degreesToRadians(pose.headingDegrees);
  const double sinHeading = std::sin(headingRadians);
  const double cosHeading = std::cos(headingRadians);
  const double deltaX = point.xInches - pose.xInches;
  const double deltaY = point.yInches - pose.yInches;
  return {deltaX * cosHeading - deltaY * sinHeading, deltaX * sinHeading + deltaY * cosHeading};
}

}  // namespace cardLib
