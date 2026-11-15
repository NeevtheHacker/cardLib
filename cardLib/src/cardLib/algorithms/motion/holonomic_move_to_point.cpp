// holonomic_move_to_point.cpp
// Why: implements the holonomic moveToPoint from SPEC 20.10.
// Owns: nothing beyond HolonomicMoveToPoint's own state.
#include "cardLib/algorithms/motion/holonomic_move_to_point.hpp"

#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

namespace {
// Closer than this, the direction to the point is meaningless; translation stops.
constexpr double MIN_DIRECTION_DISTANCE_INCHES = 1e-6;
}  // namespace

TranslationChannels translationToward(const Pose& pose, const Point& fieldDirection, double speed,
                                      const DriveKinematics& kinematics, double maxSpeed) {
  const LocalOffset local = fieldToLocal({0.0, 0.0, pose.headingDegrees}, fieldDirection);
  const double maxStrafe = kinematics.maxStrafeInchesPerSecond();
  const double strafeScale =
      maxStrafe > 0.0 ? kinematics.maxForwardInchesPerSecond() / maxStrafe : 0.0;
  return {{speed * local.forwardInches, local.forwardInches, {.maxSpeed = maxSpeed}},
          {speed * local.rightInches * strafeScale, local.rightInches, {.maxSpeed = MAX_SPEED}}};
}

HolonomicMoveToPoint::HolonomicMoveToPoint(const HolonomicMoveToPointSettings& settings)
    : m_settings(settings),
      m_translationPid(settings.translationGains),
      m_headingPid(settings.headingGains, ErrorKind::Angle),
      m_exits(ExitSettings{}) {}

void HolonomicMoveToPoint::configure(const Point& target, const MotionOptions& options) {
  m_target = target;
  m_options = options;
}

void HolonomicMoveToPoint::setSettings(const HolonomicMoveToPointSettings& settings) {
  m_settings = settings;
  m_translationPid.setGains(settings.translationGains);
  m_headingPid.setGains(settings.headingGains);
}

void HolonomicMoveToPoint::start(const RobotState& state) {
  m_headingDegrees = m_options.headingDegrees.value_or(state.pose.headingDegrees);
  m_lastPose = state.pose;
  m_travelledInches = 0.0;
  m_translationPid.reset();
  m_headingPid.reset();
  m_exits.reset(makeExitSettings(m_settings.exitWindows, m_options, m_settings.useCurrentForStall));
}

double HolonomicMoveToPoint::distanceToTargetInches(const Pose& pose) const {
  return std::hypot(m_target.xInches - pose.xInches, m_target.yInches - pose.yInches);
}

double HolonomicMoveToPoint::headingErrorDegrees(const Pose& pose) const {
  return turnErrorDegrees(m_headingDegrees, pose.headingDegrees);
}

MotionStep HolonomicMoveToPoint::step(const RobotState& state, const MotionTick& tick,
                                      DrivePipeline& pipeline) {
  const double distance = distanceToTargetInches(state.pose);
  const double speed = limitSpeed({m_translationPid.update(distance, tick.dtSeconds), distance,
                                   mainChannelLimits(m_options, m_settings.exitWindows)});
  Point direction{};
  if (distance > MIN_DIRECTION_DISTANCE_INCHES) {
    direction = {(m_target.xInches - state.pose.xInches) / distance,
                 (m_target.yInches - state.pose.yInches) / distance};
  }
  const TranslationChannels translation =
      translationToward(state.pose, direction, speed, pipeline.kinematics(), m_options.maxSpeed);
  const double headingError = headingErrorDegrees(state.pose);
  const ChannelRequest angular{m_headingPid.update(headingError, tick.dtSeconds), headingError,
                               correctionChannelLimits(m_options)};

  m_travelledInches +=
      std::hypot(state.pose.xInches - m_lastPose.xInches, state.pose.yInches - m_lastPose.yInches);
  m_lastPose = state.pose;

  const double speedInchesPerSecond =
      std::hypot(state.linearVelocityInchesPerSecond, state.sidewaysVelocityInchesPerSecond);
  MotionStep result;
  result.command =
      pipeline.updateHolonomic(translation.forward, translation.strafe, angular, tick.dtSeconds);
  result.exit = m_exits.check(
      makeExitInputs(distance, speedInchesPerSecond, state, tick, m_options.allowDegraded));
  result.progress = m_travelledInches;
  result.target = 0.0;
  result.measured = distance;
  return result;
}

}  // namespace cardLib
