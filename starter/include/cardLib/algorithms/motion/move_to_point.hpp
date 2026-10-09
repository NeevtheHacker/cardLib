// move_to_point.hpp
// Why: moveToPoint(x, y): drive to a field point, turning toward it as it goes (SPEC section 8).
// Owns: the PIDs, exit timers, target, frozen heading and distance travelled.
#pragma once

#include "cardLib/algorithms/control/exit_conditions.hpp"
#include "cardLib/algorithms/control/pid_controller.hpp"
#include "cardLib/algorithms/interfaces/motion_algorithm.hpp"
#include "cardLib/algorithms/math/pose.hpp"
#include "cardLib/algorithms/motion/motion_options.hpp"

namespace cardLib {

struct MoveToPointSettings {
  PidGains linearGains = {};
  PidGains headingGains = {};
  ExitWindows exitWindows = MOVE_EXIT_WINDOWS;
  bool useCurrentForStall = false;
  double settleRadiusInches = 6.0;  // inside this, the heading target freezes (tune)
};

class MoveToPoint : public MotionAlgorithm {
 public:
  explicit MoveToPoint(const MoveToPointSettings& settings);

  void configure(const Point& target, const MotionOptions& options);
  void setSettings(const MoveToPointSettings& settings);

  void start(const RobotState& state) override;
  MotionStep step(const RobotState& state, const MotionTick& tick,
                  DrivePipeline& pipeline) override;

  double distanceToTargetInches(const Pose& pose) const;

 private:
  void updateTargetHeading(const Pose& pose, double distanceInches);

  MoveToPointSettings m_settings;
  MotionOptions m_options;
  Point m_target;
  double m_targetHeadingDegrees = 0.0;
  bool m_hasTargetHeading = false;
  Pose m_lastPose;
  double m_travelledInches = 0.0;
  PidController m_linearPid;
  PidController m_headingPid;
  ExitConditions m_exits;
};

}  // namespace cardLib
