// strafe.hpp
// Why: strafe(d): slide sideways a relative distance while holding the starting heading, the
//      mirror of move (SPEC 20.10). Holonomic drives only.
// Owns: the strafe's PIDs, exit timers, start pose and target.
#pragma once

#include "cardLib/algorithms/control/exit_conditions.hpp"
#include "cardLib/algorithms/control/pid_controller.hpp"
#include "cardLib/algorithms/interfaces/motion_algorithm.hpp"
#include "cardLib/algorithms/motion/motion_options.hpp"

namespace cardLib {

struct StrafeSettings {
  PidGains strafeGains = {};
  PidGains headingGains = {};
  ExitWindows exitWindows = MOVE_EXIT_WINDOWS;
  bool useCurrentForStall = false;
};

// r = (x - x0) cos(theta0) - (y - y0) sin(theta0): progress to the right of the start heading.
double strafeProgressInches(const Pose& start, const Pose& current);

class Strafe : public MotionAlgorithm {
 public:
  explicit Strafe(const StrafeSettings& settings);

  // Positive distance slides right, negative left. Call before start().
  void configure(double distanceInches, const MotionOptions& options);
  void setSettings(const StrafeSettings& settings);

  void start(const RobotState& state) override;
  MotionStep step(const RobotState& state, const MotionTick& tick,
                  DrivePipeline& pipeline) override;

 private:
  StrafeSettings m_settings;
  MotionOptions m_options;
  double m_targetInches = 0.0;
  Pose m_start;
  PidController m_strafePid;
  PidController m_headingPid;
  ExitConditions m_exits;
};

}  // namespace cardLib
