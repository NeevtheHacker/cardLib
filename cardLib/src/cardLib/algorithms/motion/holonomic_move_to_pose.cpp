// holonomic_move_to_pose.cpp
// Why: implements the holonomic moveToPose from SPEC 20.10.
// Owns: nothing beyond HolonomicMoveToPose's own state.
#include "cardLib/algorithms/motion/holonomic_move_to_pose.hpp"

#include <cstdint>
#include <limits>

namespace cardLib {

HolonomicMoveToPose::HolonomicMoveToPose(const HolonomicMoveToPoseSettings& settings)
    : m_settings(settings), m_drive(settings.drive), m_headingExits(ExitSettings{}) {}

void HolonomicMoveToPose::configure(const Pose& target, const MotionOptions& options) {
  m_options = options;
  m_options.headingDegrees = target.headingDegrees;
  m_drive.configure({target.xInches, target.yInches}, m_options);
}

void HolonomicMoveToPose::setSettings(const HolonomicMoveToPoseSettings& settings) {
  m_settings = settings;
  m_drive.setSettings(settings.drive);
}

void HolonomicMoveToPose::start(const RobotState& state) {
  m_drive.start(state);
  // Only its Settled answer is used; failures and timeouts come from the translation.
  m_headingExits.reset({.windows = m_settings.headingWindows,
                        .timeoutMs = std::numeric_limits<std::uint32_t>::max()});
}

bool HolonomicMoveToPose::headingSettled(const RobotState& state, const MotionTick& tick) {
  const std::optional<ExitDecision> decision =
      m_headingExits.check({.error = m_drive.headingErrorDegrees(state.pose),
                            .velocity = state.angularVelocityDegreesPerSecond,
                            .elapsedMs = tick.elapsedMs});
  return decision.has_value() && decision->result == MotionResult::Settled;
}

MotionStep HolonomicMoveToPose::step(const RobotState& state, const MotionTick& tick,
                                     DrivePipeline& pipeline) {
  const bool headingDone = headingSettled(state, tick);
  MotionStep result = m_drive.step(state, tick, pipeline);
  const bool translationSettled =
      result.exit.has_value() && result.exit->result == MotionResult::Settled;
  if (translationSettled && !headingDone) {
    result.exit.reset();
  }
  return result;
}

}  // namespace cardLib
