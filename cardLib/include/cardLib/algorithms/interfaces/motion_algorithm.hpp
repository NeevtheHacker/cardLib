// motion_algorithm.hpp
// Why: one interface for every kind of motion, so the motion task runs them all the same way.
// Owns: MotionAlgorithm, MotionTick, MotionStep and the shared exit-input helper (SPEC 8, 18).
#pragma once

#include <cstdint>
#include <optional>

#include "cardLib/algorithms/control/drive_command.hpp"
#include "cardLib/algorithms/control/drive_pipeline.hpp"
#include "cardLib/algorithms/control/exit_conditions.hpp"
#include "cardLib/algorithms/interfaces/robot_state.hpp"
#include "cardLib/algorithms/math/loop_timing.hpp"
#include "cardLib/algorithms/motion/motion_options.hpp"

namespace cardLib {

// Everything about this loop that isn't the robot's state.
struct MotionTick {
  double dtSeconds = LOOP_PERIOD_SECONDS;
  std::uint32_t elapsedMs = 0;      // since the motion started
  bool calibrated = true;           // startup validation passed
  bool cancelled = false;           // cancel() or driver control took over
  bool driveFault = false;          // a required drive motor stopped answering
  bool currentConfirmsStall = false;  // see currentConfirmsStall(); read only if enabled
};

// What the motion wants this loop. Once exit is set, the motion is over and command is ignored.
struct MotionStep {
  DriveCommand command = {};
  std::optional<ExitDecision> exit = std::nullopt;
  double progress = 0.0;  // inches for moves, degrees for turns; what waitUntil() counts
};

class MotionAlgorithm {
 public:
  virtual ~MotionAlgorithm() = default;

  // Resets PIDs and exit timers and records the start state. Called once before the first step.
  virtual void start(const RobotState& state) = 0;

  // One 10 ms loop: read the state, compute errors, run the shared drive pipeline, check exits.
  virtual MotionStep step(const RobotState& state, const MotionTick& tick,
                          DrivePipeline& pipeline) = 0;
};

// Builds the exit inputs every motion shares. Odometry that is faulted, or degraded without
// allowDegraded, counts as a sensor fault (SPEC section 4).
ExitInputs makeExitInputs(double error, double velocity, const RobotState& state,
                          const MotionTick& tick, bool allowDegraded);

// Exit settings for one motion: its windows, plus earlyExit and timeoutMs from the options.
ExitSettings makeExitSettings(const ExitWindows& windows, const MotionOptions& options,
                              bool useCurrentForStall);

// The limits for the channel that carries the motion's main error. minSpeed stops applying
// inside earlyExit, or inside the small exit window when earlyExit is off (SPEC section 7).
SpeedLimits mainChannelLimits(const MotionOptions& options, const ExitWindows& windows);

// The limits for a correction channel (heading hold): maxSpeed only, never minSpeed.
SpeedLimits correctionChannelLimits(const MotionOptions& options);

}  // namespace cardLib
