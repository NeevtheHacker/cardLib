// holonomic_move_to_pose.hpp
// Why: moveToPose on a holonomic drive: translate to the point and turn to the heading at the
//      same time, with no hand-off phase (SPEC 20.10).
// Owns: the translation motion and the heading exit timers.
#pragma once

#include "cardLib/algorithms/control/exit_conditions.hpp"
#include "cardLib/algorithms/interfaces/motion_algorithm.hpp"
#include "cardLib/algorithms/math/pose.hpp"
#include "cardLib/algorithms/motion/holonomic_move_to_point.hpp"
#include "cardLib/algorithms/motion/motion_options.hpp"

namespace cardLib {

struct HolonomicMoveToPoseSettings {
  HolonomicMoveToPointSettings drive = {};  // its headingGains turn the robot (turn gains)
  ExitWindows headingWindows = TURN_EXIT_WINDOWS;
};

class HolonomicMoveToPose : public MotionAlgorithm {
 public:
  explicit HolonomicMoveToPose(const HolonomicMoveToPoseSettings& settings);

  void configure(const Pose& target, const MotionOptions& options);
  void setSettings(const HolonomicMoveToPoseSettings& settings);

  void start(const RobotState& state) override;

  // Every exit comes from the translation, except that it can't settle until the heading has
  // settled in its own (turn) window too.
  MotionStep step(const RobotState& state, const MotionTick& tick,
                  DrivePipeline& pipeline) override;

 private:
  bool headingSettled(const RobotState& state, const MotionTick& tick);

  HolonomicMoveToPoseSettings m_settings;
  MotionOptions m_options;
  HolonomicMoveToPoint m_drive;
  ExitConditions m_headingExits;
};

}  // namespace cardLib
