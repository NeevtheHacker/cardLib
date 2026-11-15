// strafe.cpp
// Why: implements strafe(d) from SPEC 20.10, the sideways mirror of move.
// Owns: nothing beyond Strafe's own state.
#include "cardLib/algorithms/motion/strafe.hpp"

#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"
#include "cardLib/algorithms/math/pose.hpp"

namespace cardLib {

double strafeProgressInches(const Pose& start, const Pose& current) {
  const Pose startFacing{0.0, 0.0, start.headingDegrees};
  const Point moved{current.xInches - start.xInches, current.yInches - start.yInches};
  return fieldToLocal(startFacing, moved).rightInches;
}

Strafe::Strafe(const StrafeSettings& settings)
    : m_settings(settings),
      m_strafePid(settings.strafeGains),
      m_headingPid(settings.headingGains, ErrorKind::Angle),
      m_exits(ExitSettings{}) {}

void Strafe::configure(double distanceInches, const MotionOptions& options) {
  m_options = options;
  m_targetInches = distanceInches;
}

void Strafe::setSettings(const StrafeSettings& settings) {
  m_settings = settings;
  m_strafePid.setGains(settings.strafeGains);
  m_headingPid.setGains(settings.headingGains);
}

void Strafe::start(const RobotState& state) {
  m_start = state.pose;
  m_strafePid.reset();
  m_headingPid.reset();
  m_exits.reset(makeExitSettings(m_settings.exitWindows, m_options, m_settings.useCurrentForStall));
}

MotionStep Strafe::step(const RobotState& state, const MotionTick& tick, DrivePipeline& pipeline) {
  const double progress = strafeProgressInches(m_start, state.pose);
  const double strafeError = m_targetInches - progress;
  const double headingError = turnErrorDegrees(m_start.headingDegrees, state.pose.headingDegrees);

  const ChannelRequest linear{0.0, 0.0, correctionChannelLimits(m_options)};
  const ChannelRequest strafe{m_strafePid.update(strafeError, tick.dtSeconds), strafeError,
                              mainChannelLimits(m_options, m_settings.exitWindows)};
  const ChannelRequest angular{m_headingPid.update(headingError, tick.dtSeconds), headingError,
                               correctionChannelLimits(m_options)};

  MotionStep result;
  result.command = pipeline.updateHolonomic(linear, strafe, angular, tick.dtSeconds);
  result.exit = m_exits.check(makeExitInputs(strafeError, state.sidewaysVelocityInchesPerSecond,
                                             state, tick, m_options.allowDegraded));
  result.progress = std::abs(progress);
  result.target = m_targetInches;
  result.measured = progress;
  return result;
}

}  // namespace cardLib
