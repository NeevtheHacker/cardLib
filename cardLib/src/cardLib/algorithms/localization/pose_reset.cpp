// pose_reset.cpp
// Why: implements the SPEC 21 wall and GPS resets and their refusal rules.
// Owns: nothing; no state.
#include "cardLib/algorithms/localization/pose_reset.hpp"

#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

namespace {

// How one wall is described: the beam direction that points straight at it, which field axis
// it fixes, where it is, and which way "in from the wall" goes along that axis.
struct WallGeometry {
  double towardDegrees = 0.0;
  bool fixesX = true;
  double positionInches = 0.0;
  double inward = 1.0;
};

WallGeometry wallGeometry(Wall wall, const FieldWalls& walls) {
  switch (wall) {
    case Wall::Left:
      return {-90.0, true, walls.leftXInches, 1.0};
    case Wall::Right:
      return {90.0, true, walls.rightXInches, -1.0};
    case Wall::Back:
      return {180.0, false, walls.backYInches, 1.0};
    case Wall::Front:
      return {0.0, false, walls.frontYInches, -1.0};
  }
  return {};
}

bool stopped(const RobotState& state, double maxSpeed, double maxTurnRate) {
  return std::hypot(state.linearVelocityInchesPerSecond, state.sidewaysVelocityInchesPerSecond) <=
             maxSpeed &&
         std::abs(state.angularVelocityDegreesPerSecond) <= maxTurnRate;
}

PoseResetDecision refuse(const RobotState& state, ResetResult result) {
  return {result, state.pose, 0.0};
}

}  // namespace

PoseResetDecision wallReset(const RobotState& state, const DistanceMount& mount,
                            const DistanceReading& reading, Wall wall, const FieldWalls& walls,
                            const WallResetOptions& options) {
  if (!reading.valid) {
    return refuse(state, ResetResult::NoReading);
  }
  if (reading.inches < options.minRangeInches || reading.inches > options.maxRangeInches) {
    return refuse(state, ResetResult::OutOfRange);
  }
  if (!stopped(state, options.maxSpeedInchesPerSecond, options.maxTurnRateDegreesPerSecond)) {
    return refuse(state, ResetResult::NotStopped);
  }
  const WallGeometry geometry = wallGeometry(wall, walls);
  const double beamDegrees = state.pose.headingDegrees + mount.headingDegrees;
  const double offSquareDegrees = wrapDegrees(beamDegrees - geometry.towardDegrees);
  if (std::abs(offSquareDegrees) > options.maxAngleFromSquareDegrees) {
    return refuse(state, ResetResult::NotSquare);
  }

  const double fromWallInches = reading.inches * std::cos(degreesToRadians(offSquareDegrees));
  const Point sensorOffset =
      localToField({0.0, 0.0, state.pose.headingDegrees}, {mount.xInches, mount.yInches});
  const double sensorCoordinate = geometry.positionInches + geometry.inward * fromWallInches;
  PoseResetDecision decision{ResetResult::Applied, state.pose, 0.0};
  double& coordinate = geometry.fixesX ? decision.pose.xInches : decision.pose.yInches;
  const double offset = geometry.fixesX ? sensorOffset.xInches : sensorOffset.yInches;
  coordinate = sensorCoordinate - offset;
  decision.changeInches =
      std::abs(coordinate - (geometry.fixesX ? state.pose.xInches : state.pose.yInches));
  if (decision.changeInches > options.maxChangeInches) {
    return refuse(state, ResetResult::ChangeTooLarge);
  }
  return decision;
}

Pose robotPoseFromGps(const GpsSensorPose& sensor, const GpsMount& mount) {
  const Point turned =
      localToField({0.0, 0.0, mount.fieldRotationDegrees}, {sensor.xInches, sensor.yInches});
  const double sensorX = turned.xInches + mount.fieldXOffsetInches;
  const double sensorY = turned.yInches + mount.fieldYOffsetInches;
  const double robotHeading =
      sensor.headingDegrees + mount.fieldRotationDegrees - mount.headingDegrees;
  const Point mountOffset = localToField({0.0, 0.0, robotHeading}, {mount.xInches, mount.yInches});
  return {sensorX - mountOffset.xInches, sensorY - mountOffset.yInches, robotHeading};
}

PoseResetDecision gpsReset(const RobotState& state, const GpsReading& reading,
                           const GpsResetOptions& options) {
  if (!reading.valid) {
    return refuse(state, ResetResult::NoReading);
  }
  if (reading.qualityPercent.has_value() && *reading.qualityPercent < options.minQualityPercent) {
    return refuse(state, ResetResult::LowQuality);
  }
  if (reading.errorInches > options.maxErrorInches) {
    return refuse(state, ResetResult::ErrorTooHigh);
  }
  if (!stopped(state, options.maxSpeedInchesPerSecond, options.maxTurnRateDegreesPerSecond)) {
    return refuse(state, ResetResult::NotStopped);
  }
  PoseResetDecision decision{ResetResult::Applied, state.pose, 0.0};
  decision.pose.xInches = reading.robotPose.xInches;
  decision.pose.yInches = reading.robotPose.yInches;
  if (options.resetHeading) {
    decision.pose.headingDegrees =
        state.pose.headingDegrees +
        wrapDegrees(reading.robotPose.headingDegrees - state.pose.headingDegrees);
  }
  decision.changeInches = std::hypot(decision.pose.xInches - state.pose.xInches,
                                     decision.pose.yInches - state.pose.yInches);
  return decision;
}

}  // namespace cardLib
