// turn.cpp
// Why: implements turn() and turnToPoint() from SPEC section 8.
// Owns: nothing beyond Turn's own state.
#include "cardLib/algorithms/motion/turn.hpp"

#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

Turn::Turn(const TurnSettings& settings)
    : m_settings(settings), m_pid(settings.turnGains, ErrorKind::Angle), m_exits(ExitSettings{}) {}

void Turn::configureHeading(double headingDegrees, const MotionOptions& options) {
  m_kind = TargetKind::Heading;
  m_targetHeadingDegrees = headingDegrees;
  m_options = options;
}

void Turn::configurePoint(const Point& target, const MotionOptions& options) {
  m_kind = TargetKind::Point;
  m_targetPoint = target;
  m_options = options;
}

void Turn::setSettings(const TurnSettings& settings) {
  m_settings = settings;
  m_pid.setGains(settings.turnGains);
}

void Turn::start(const RobotState& state) {
  m_startHeadingDegrees = state.pose.headingDegrees;
  m_direction = m_options.direction;
  if (m_kind == TargetKind::Point) {
    m_targetHeadingDegrees = state.pose.headingDegrees;
  }
  m_pid.reset();
  m_exits.reset(
      makeExitSettings(m_settings.exitWindows, m_options, m_settings.useCurrentForStall));
}

void Turn::updateTargetHeading(const Pose& pose) {
  if (m_kind != TargetKind::Point) {
    return;
  }
  const double distance =
      std::hypot(m_targetPoint.xInches - pose.xInches, m_targetPoint.yInches - pose.yInches);
  if (distance < TURN_TO_POINT_MIN_DISTANCE_INCHES) {
    return;
  }
  const double facing = headingToPointDegrees(pose.xInches, pose.yInches, m_targetPoint.xInches,
                                              m_targetPoint.yInches);
  m_targetHeadingDegrees = m_options.forwards ? facing : reverseHeadingDegrees(facing);
}

MotionStep Turn::step(const RobotState& state, const MotionTick& tick, DrivePipeline& pipeline) {
  updateTargetHeading(state.pose);
  const double error =
      directedTurnErrorDegrees(m_targetHeadingDegrees, state.pose.headingDegrees, m_direction);

  const ChannelRequest linear{0.0, 0.0, correctionChannelLimits(m_options)};
  const ChannelRequest angular{m_pid.update(error, tick.dtSeconds), error,
                               mainChannelLimits(m_options, m_settings.exitWindows)};

  MotionStep result;
  result.command = pipeline.update(linear, angular, tick.dtSeconds);
  result.exit = m_exits.check(makeExitInputs(error, state.angularVelocityDegreesPerSecond, state,
                                             tick, m_options.allowDegraded));
  result.progress = std::abs(state.pose.headingDegrees - m_startHeadingDegrees);
  result.target = m_targetHeadingDegrees;
  result.measured = state.pose.headingDegrees;
  return result;
}

}  // namespace cardLib
