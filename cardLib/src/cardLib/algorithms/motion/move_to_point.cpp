// move_to_point.cpp
// Why: implements moveToPoint() from SPEC section 8, with the single projection.
// Owns: nothing beyond MoveToPoint's own state.
#include "cardLib/algorithms/motion/move_to_point.hpp"

#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

MoveToPoint::MoveToPoint(const MoveToPointSettings& settings)
    : m_settings(settings),
      m_linearPid(settings.linearGains),
      m_headingPid(settings.headingGains, ErrorKind::Angle),
      m_exits(ExitSettings{}) {}

void MoveToPoint::configure(const Point& target, const MotionOptions& options) {
  m_target = target;
  m_options = options;
}

void MoveToPoint::setSettings(const MoveToPointSettings& settings) {
  m_settings = settings;
  m_linearPid.setGains(settings.linearGains);
  m_headingPid.setGains(settings.headingGains);
}

void MoveToPoint::start(const RobotState& state) {
  m_hasTargetHeading = false;
  m_lastPose = state.pose;
  m_travelledInches = 0.0;
  m_linearPid.reset();
  m_headingPid.reset();
  m_exits.reset(makeExitSettings(m_settings.exitWindows, m_options, m_settings.useCurrentForStall));
}

double MoveToPoint::distanceToTargetInches(const Pose& pose) const {
  return std::hypot(m_target.xInches - pose.xInches, m_target.yInches - pose.yInches);
}

// Near the target a tiny sideways error swings the heading to it a lot, so it freezes.
void MoveToPoint::updateTargetHeading(const Pose& pose, double distanceInches) {
  if (m_hasTargetHeading && distanceInches <= m_settings.settleRadiusInches) {
    return;
  }
  const double facing =
      headingToPointDegrees(pose.xInches, pose.yInches, m_target.xInches, m_target.yInches);
  m_targetHeadingDegrees = m_options.forwards ? facing : reverseHeadingDegrees(facing);
  m_hasTargetHeading = true;
}

MotionStep MoveToPoint::step(const RobotState& state, const MotionTick& tick,
                             DrivePipeline& pipeline) {
  const double distance = distanceToTargetInches(state.pose);
  updateTargetHeading(state.pose, distance);

  // Signed distance remaining along the robot's heading; already distance * cos(e_ang).
  const double linearError = fieldToLocal(state.pose, m_target).forwardInches;
  const double headingError = turnErrorDegrees(m_targetHeadingDegrees, state.pose.headingDegrees);

  const ChannelRequest linear{m_linearPid.update(linearError, tick.dtSeconds), linearError,
                              mainChannelLimits(m_options, m_settings.exitWindows)};
  const ChannelRequest angular{m_headingPid.update(headingError, tick.dtSeconds), headingError,
                               correctionChannelLimits(m_options)};

  m_travelledInches +=
      std::hypot(state.pose.xInches - m_lastPose.xInches, state.pose.yInches - m_lastPose.yInches);
  m_lastPose = state.pose;

  MotionStep result;
  result.command = pipeline.update(linear, angular, tick.dtSeconds);
  result.exit = m_exits.check(makeExitInputs(distance, state.linearVelocityInchesPerSecond, state,
                                             tick, m_options.allowDegraded));
  result.progress = m_travelledInches;
  result.target = 0.0;
  result.measured = distance;
  return result;
}

}  // namespace cardLib
