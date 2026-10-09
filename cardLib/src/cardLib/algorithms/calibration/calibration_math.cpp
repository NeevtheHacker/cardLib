// calibration_math.cpp
// Why: implements the SPEC section 6 calibration formulas.
// Owns: nothing; no state.
#include "cardLib/algorithms/calibration/calibration_math.hpp"

#include <algorithm>
#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

namespace {

std::optional<double> divide(double numerator, double denominator) {
  if (denominator == 0.0) {
    return std::nullopt;
  }
  return numerator / denominator;
}

std::optional<double> centroidDistance(std::span<const Point> endErrors) {
  if (endErrors.empty()) {
    return std::nullopt;
  }
  double xSum = 0.0;
  double ySum = 0.0;
  for (const Point& error : endErrors) {
    xSum += error.xInches;
    ySum += error.yInches;
  }
  const double count = static_cast<double>(endErrors.size());
  return std::hypot(xSum / count, ySum / count);
}

}  // namespace

std::optional<double> imuScale(double turns, double clockwiseRotationDegrees,
                               double counterclockwiseRotationDegrees) {
  const double expectedDegrees = 360.0 * turns;
  const auto clockwise = divide(expectedDegrees, std::abs(clockwiseRotationDegrees));
  const auto counterclockwise = divide(expectedDegrees, std::abs(counterclockwiseRotationDegrees));
  if (!clockwise.has_value() || !counterclockwise.has_value()) {
    return std::nullopt;
  }
  return (*clockwise + *counterclockwise) / 2.0;
}

std::optional<double> trackingWheelDiameter(double nominalDiameterInches, double tapedInches,
                                            double measuredInches) {
  return divide(nominalDiameterInches * tapedInches, measuredInches);
}

std::optional<double> driveScale(double tapedInches, double driveMeasuredInches) {
  return divide(tapedInches, driveMeasuredInches);
}

std::optional<double> verticalWheelOffset(double wheelInches, double headingChangeRadians) {
  return divide(-wheelInches, headingChangeRadians);
}

std::optional<double> horizontalWheelOffset(double wheelInches, double headingChangeRadians) {
  return divide(wheelInches, headingChangeRadians);
}

std::optional<double> effectiveTrackWidth(double leftInches, double rightInches,
                                          double headingChangeRadians) {
  return divide(leftInches - rightInches, headingChangeRadians);
}

double medianOfThree(double a, double b, double c) {
  return std::max(std::min(a, b), std::min(std::max(a, b), c));
}

bool scalesAgree(double first, double second, double toleranceFraction) {
  return std::abs(first - second) <= toleranceFraction * std::abs(first);
}

bool offsetsAgree(double firstInches, double secondInches, double toleranceInches) {
  return std::abs(firstInches - secondInches) <= toleranceInches;
}

TapeBoxError tapeBoxError(const Pose& start, const Pose& end) {
  return {std::hypot(end.xInches - start.xInches, end.yInches - start.yInches),
          wrapDegrees(end.headingDegrees - start.headingDegrees)};
}

std::optional<double> umbmarkScore(std::span<const Point> clockwiseEndErrors,
                                   std::span<const Point> counterclockwiseEndErrors) {
  const auto clockwise = centroidDistance(clockwiseEndErrors);
  const auto counterclockwise = centroidDistance(counterclockwiseEndErrors);
  if (!clockwise.has_value() || !counterclockwise.has_value()) {
    return std::nullopt;
  }
  return std::max(*clockwise, *counterclockwise);
}

}  // namespace cardLib
