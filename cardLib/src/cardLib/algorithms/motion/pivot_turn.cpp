// pivot_turn.cpp
// Why: implements pivotTurn() from SPEC section 8.
// Owns: nothing beyond PivotTurn's own state.
#include "cardLib/algorithms/motion/pivot_turn.hpp"

#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

PivotTurn::PivotTurn(const PivotTurnSettings& settings)
    : m_settings(settings), m_pid(settings.pivotGains, ErrorKind::Angle), m_exits(ExitSettings{}) {}

void PivotTurn::configure(double headingDegrees, Side lockedSide, const MotionOptions& options) {
  m_targetHeadingDegrees = headingDegrees;
  m_lockedSide = lockedSide;
  m_aboutPoint = false;
  m_options = options;
}

void PivotTurn::configure(double headingDegrees, const LocalOffset& pivot,
                          const MotionOptions& options) {
  m_targetHeadingDegrees = headingDegrees;
  m_pivot = pivot;
  m_aboutPoint = true;
  m_options = options;
}

void PivotTurn::setSettings(const PivotTurnSettings& settings) {
  m_settings = settings;
  m_pid.setGains(settings.pivotGains);
}

void PivotTurn::start(const RobotState& state) {
  m_startHeadingDegrees = state.pose.headingDegrees;
  m_direction = m_options.direction;
  m_pid.reset();
  m_exits.reset(makeExitSettings(m_settings.exitWindows, m_options, m_settings.useCurrentForStall));
}

MotionStep PivotTurn::step(const RobotState& state, const MotionTick& tick,
                           DrivePipeline& pipeline) {
  const double error =
      directedTurnErrorDegrees(m_targetHeadingDegrees, state.pose.headingDegrees, m_direction);
  const ChannelRequest turn{m_pid.update(error, tick.dtSeconds), error,
                            mainChannelLimits(m_options, m_settings.exitWindows)};

  MotionStep result;
  result.command = m_aboutPoint ? pipeline.updatePivotAbout(turn, m_pivot, tick.dtSeconds)
                                : pipeline.updatePivot(turn, m_lockedSide, tick.dtSeconds);
  result.exit = m_exits.check(makeExitInputs(error, state.angularVelocityDegreesPerSecond, state,
                                             tick, m_options.allowDegraded));
  result.progress = std::abs(state.pose.headingDegrees - m_startHeadingDegrees);
  result.target = m_targetHeadingDegrees;
  result.measured = state.pose.headingDegrees;
  return result;
}

}  // namespace cardLib
