// motion_algorithm.cpp
// Why: the exit-input rules every motion shares, written once.
// Owns: nothing; no state.
#include "cardLib/algorithms/interfaces/motion_algorithm.hpp"

namespace cardLib {

ExitInputs makeExitInputs(double error, double velocity, const RobotState& state,
                          const MotionTick& tick, bool allowDegraded) {
  const bool odometryRefused = state.health == OdometryHealth::Faulted ||
                               (state.health == OdometryHealth::Degraded && !allowDegraded);
  return {.error = error,
          .velocity = velocity,
          .elapsedMs = tick.elapsedMs,
          .calibrated = tick.calibrated,
          .sensorFault = tick.driveFault || odometryRefused,
          .cancelled = tick.cancelled,
          .currentConfirmsStall = tick.currentConfirmsStall};
}

ExitSettings makeExitSettings(const ExitWindows& windows, const MotionOptions& options,
                              bool useCurrentForStall) {
  return {.windows = windows,
          .earlyExit = options.earlyExit,
          .timeoutMs = options.timeoutMs.value_or(DEFAULT_TIMEOUT_MS),
          .useCurrentForStall = useCurrentForStall};
}

SpeedLimits mainChannelLimits(const MotionOptions& options, const ExitWindows& windows) {
  return {.maxSpeed = options.maxSpeed,
          .minSpeed = options.minSpeed,
          .minSpeedCutoff = options.earlyExit > 0.0 ? options.earlyExit : windows.smallError};
}

SpeedLimits correctionChannelLimits(const MotionOptions& options) {
  return {.maxSpeed = options.maxSpeed};
}

}  // namespace cardLib
