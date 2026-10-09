// calibration_math.hpp
// Why: the SPEC section 6 calibration formulas, kept apart from the routines that run the robot.
// Owns: nothing; every function is pure. Empty results mean the data can't give an answer.
#pragma once

#include <optional>
#include <span>

#include "cardLib/algorithms/math/pose.hpp"

namespace cardLib {

// 1. k_imu = 360 * N / measured rotation, averaged over a clockwise and a counterclockwise run.
std::optional<double> imuScale(double turns, double clockwiseRotationDegrees,
                               double counterclockwiseRotationDegrees);

// 2. D_true = D_nom * L / d_measured.
std::optional<double> trackingWheelDiameter(double nominalDiameterInches, double tapedInches,
                                            double measuredInches);

// 3. s_drive = L / d_drive.
std::optional<double> driveScale(double tapedInches, double driveMeasuredInches);

// 4. From a pure spin (total IMU change in radians): s_v = -dv / dtheta, s_h = dh / dtheta.
std::optional<double> verticalWheelOffset(double wheelInches, double headingChangeRadians);
std::optional<double> horizontalWheelOffset(double wheelInches, double headingChangeRadians);

// 5. W_eff = (dL - dR) / dtheta, from the same spin.
std::optional<double> effectiveTrackWidth(double leftInches, double rightInches,
                                          double headingChangeRadians);

double medianOfThree(double a, double b, double c);

// Pass criteria: two runs agree within 0.5 % (scales) or 0.1 in (offsets). (tune)
bool scalesAgree(double first, double second, double toleranceFraction = 0.005);
bool offsetsAgree(double firstInches, double secondInches, double toleranceInches = 0.1);

struct TapeBoxError {
  double positionInches = 0.0;
  double headingDegrees = 0.0;
};

// How far the pose ended from where it started, after rolling back into the taped box.
TapeBoxError tapeBoxError(const Pose& start, const Pose& end);

// UMBmark: per direction, the distance of the end errors' centroid from zero; the score is the
// larger of the two. Empty if either direction has no runs.
std::optional<double> umbmarkScore(std::span<const Point> clockwiseEndErrors,
                                   std::span<const Point> counterclockwiseEndErrors);

}  // namespace cardLib
