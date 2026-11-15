// pivot_turn.hpp
// Why: pivotTurn(theta, side): turn about one locked side of the drive (SPEC section 8), or
//      about any robot-local point (SPEC 20.9).
// Owns: the pivot PID, exit timers, target and pivot point.
#pragma once

#include "cardLib/algorithms/control/drive_command.hpp"
#include "cardLib/algorithms/control/exit_conditions.hpp"
#include "cardLib/algorithms/control/pid_controller.hpp"
#include "cardLib/algorithms/interfaces/motion_algorithm.hpp"
#include "cardLib/algorithms/math/pose.hpp"
#include "cardLib/algorithms/motion/motion_options.hpp"

namespace cardLib {

struct PivotTurnSettings {
  PidGains pivotGains = {};
  ExitWindows exitWindows = PIVOT_EXIT_WINDOWS;
  bool useCurrentForStall = false;
};

class PivotTurn : public MotionAlgorithm {
 public:
  explicit PivotTurn(const PivotTurnSettings& settings);

  // lockedSide stays still with brake mode Hold; the other side does the turning.
  void configure(double headingDegrees, Side lockedSide, const MotionOptions& options);
  // Turns about this robot-local point; wheels that don't move there are held. Check the
  // drive's canPivotAbout() first.
  void configure(double headingDegrees, const LocalOffset& pivot, const MotionOptions& options);
  void setSettings(const PivotTurnSettings& settings);

  void start(const RobotState& state) override;
  MotionStep step(const RobotState& state, const MotionTick& tick,
                  DrivePipeline& pipeline) override;

 private:
  PivotTurnSettings m_settings;
  MotionOptions m_options;
  double m_targetHeadingDegrees = 0.0;
  double m_startHeadingDegrees = 0.0;
  Side m_lockedSide = Side::Left;
  bool m_aboutPoint = false;
  LocalOffset m_pivot;
  TurnDirection m_direction = TurnDirection::Shortest;
  PidController m_pid;
  ExitConditions m_exits;
};

}  // namespace cardLib
