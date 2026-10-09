// move.cpp
// Why: implements move(d) from SPEC section 8.
// Owns: nothing beyond Move's own state.
#include "cardLib/algorithms/motion/move.hpp"

#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

double moveProgressInches(const Pose& start, const Pose& current) {
  const double startHeading = degreesToRadians(start.headingDegrees);
  return (current.xInches - start.xInches) * std::sin(startHeading) +
         (current.yInches - start.yInches) * std::cos(startHeading);
}

double signedMoveTarget(double distanceInches, bool forwards) {
  return forwards ? distanceInches : -std::abs(distanceInches);
}

Move::Move(const MoveSettings& settings)
    : m_settings(settings),
      m_linearPid(settings.linearGains),
      m_headingPid(settings.headingGains, ErrorKind::Angle),
      m_exits(ExitSettings{}) {}

void Move::configure(double distanceInches, const MotionOptions& options) {
  m_options = options;
  m_targetInches = signedMoveTarget(distanceInches, options.forwards);
}

void Move::setSettings(const MoveSettings& settings) {
  m_settings = settings;
  m_linearPid.setGains(settings.linearGains);
  m_headingPid.setGains(settings.headingGains);
}

void Move::start(const RobotState& state) {
  m_start = state.pose;
  m_linearPid.reset();
  m_headingPid.reset();
  m_exits.reset(
      makeExitSettings(m_settings.exitWindows, m_options, m_settings.useCurrentForStall));
}

MotionStep Move::step(const RobotState& state, const MotionTick& tick, DrivePipeline& pipeline) {
  const double progress = moveProgressInches(m_start, state.pose);
  const double linearError = m_targetInches - progress;
  const double headingError = turnErrorDegrees(m_start.headingDegrees, state.pose.headingDegrees);

  const ChannelRequest linear{m_linearPid.update(linearError, tick.dtSeconds), linearError,
                              mainChannelLimits(m_options, m_settings.exitWindows)};
  const ChannelRequest angular{m_headingPid.update(headingError, tick.dtSeconds), headingError,
                               correctionChannelLimits(m_options)};

  MotionStep result;
  result.command = pipeline.update(linear, angular, tick.dtSeconds);
  result.exit = m_exits.check(makeExitInputs(linearError, state.linearVelocityInchesPerSecond,
                                             state, tick, m_options.allowDegraded));
  result.progress = std::abs(progress);
  result.target = m_targetInches;
  result.measured = progress;
  return result;
}

}  // namespace cardLib
