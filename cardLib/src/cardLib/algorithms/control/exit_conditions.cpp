// exit_conditions.cpp
// Why: implements the SPEC section 9 exit checks.
// Owns: nothing beyond ExitConditions' own timers.
#include "cardLib/algorithms/control/exit_conditions.hpp"

#include <cmath>
#include <cstddef>

namespace cardLib {

bool ExitConditions::HeldTimer::heldFor(bool condition, std::uint32_t nowMs,
                                        std::uint32_t durationMs) {
  if (!condition) {
    m_active = false;
    return false;
  }
  if (!m_active) {
    m_active = true;
    m_sinceMs = nowMs;
  }
  return nowMs - m_sinceMs >= durationMs;
}

void ExitConditions::reset() {
  m_smallTimer.reset();
  m_largeTimer.reset();
  m_stallTimer.reset();
}

void ExitConditions::reset(const ExitSettings& settings) {
  m_settings = settings;
  reset();
}

std::optional<ExitDecision> ExitConditions::check(const ExitInputs& inputs) {
  const ExitWindows& windows = m_settings.windows;
  const double error = std::abs(inputs.error);
  const double speed = std::abs(inputs.velocity);
  const std::uint32_t now = inputs.elapsedMs;

  if (!inputs.calibrated) {
    return ExitDecision{MotionResult::NotCalibrated, true};
  }
  if (inputs.sensorFault) {
    return ExitDecision{MotionResult::SensorFault, true};
  }
  if (inputs.cancelled) {
    return ExitDecision{MotionResult::Cancelled, true};
  }
  if (m_settings.earlyExit > 0.0 && error < m_settings.earlyExit) {
    return ExitDecision{MotionResult::Settled, false};
  }

  const bool inSmall = error < windows.smallError && speed < windows.velocityTolerance;
  if (m_smallTimer.heldFor(inSmall, now, windows.smallTimeMs)) {
    return ExitDecision{MotionResult::Settled, true};
  }
  if (m_largeTimer.heldFor(error < windows.largeError, now, windows.largeTimeMs)) {
    return ExitDecision{MotionResult::Settled, true};
  }

  const bool currentOk = !m_settings.useCurrentForStall || inputs.currentConfirmsStall;
  const bool stalled = speed < windows.stallSpeed && error > windows.largeError && currentOk;
  if (m_stallTimer.heldFor(stalled, now, windows.stallTimeMs)) {
    return ExitDecision{MotionResult::Stalled, true};
  }
  if (now > m_settings.timeoutMs) {
    return ExitDecision{MotionResult::TimedOut, true};
  }
  return std::nullopt;
}

bool currentConfirmsStall(std::span<const double> currentsAmps, double thresholdAmps,
                          double commandedSpeed, double minCommandedSpeed) {
  if (currentsAmps.empty() || std::abs(commandedSpeed) <= minCommandedSpeed) {
    return false;
  }
  std::size_t overThreshold = 0;
  for (const double current : currentsAmps) {
    if (current > thresholdAmps) {
      ++overThreshold;
    }
  }
  return 2 * overThreshold >= currentsAmps.size();
}

}  // namespace cardLib
