// move.hpp
// Why: move(d): drive straight a relative distance while holding the starting heading (SPEC 8).
// Owns: the move's PIDs, exit timers, start pose and signed target.
#pragma once

#include "cardLib/algorithms/control/exit_conditions.hpp"
#include "cardLib/algorithms/control/pid_controller.hpp"
#include "cardLib/algorithms/interfaces/motion_algorithm.hpp"
#include "cardLib/algorithms/motion/motion_options.hpp"

namespace cardLib {

struct MoveSettings {
  PidGains linearGains = {};
  PidGains headingGains = {};
  ExitWindows exitWindows = MOVE_EXIT_WINDOWS;
  bool useCurrentForStall = false;
};

// p = (x - x0) sin(theta0) + (y - y0) cos(theta0): progress along the starting direction.
double moveProgressInches(const Pose& start, const Pose& current);

// distance if forwards, -|distance| if not. A negative distance also drives backwards.
double signedMoveTarget(double distanceInches, bool forwards);

class Move : public MotionAlgorithm {
 public:
  explicit Move(const MoveSettings& settings);

  // Sets the next move. Call before start().
  void configure(double distanceInches, const MotionOptions& options);
  void setSettings(const MoveSettings& settings);

  void start(const RobotState& state) override;
  MotionStep step(const RobotState& state, const MotionTick& tick,
                  DrivePipeline& pipeline) override;

 private:
  MoveSettings m_settings;
  MotionOptions m_options;
  double m_targetInches = 0.0;
  Pose m_start;
  PidController m_linearPid;
  PidController m_headingPid;
  ExitConditions m_exits;
};

}  // namespace cardLib
