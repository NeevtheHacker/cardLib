// holonomic_move_to_point.hpp
// Why: moveToPoint on a holonomic drive: drive straight at the point along the field error
//      vector while holding a heading, no turning toward it first (SPEC 20.10).
// Owns: the translation and heading PIDs, exit timers, target, held heading and distance
//       travelled.
#pragma once

#include "cardLib/algorithms/control/drive_kinematics.hpp"
#include "cardLib/algorithms/control/exit_conditions.hpp"
#include "cardLib/algorithms/control/pid_controller.hpp"
#include "cardLib/algorithms/interfaces/motion_algorithm.hpp"
#include "cardLib/algorithms/math/pose.hpp"
#include "cardLib/algorithms/motion/motion_options.hpp"

namespace cardLib {

struct HolonomicMoveToPointSettings {
  PidGains translationGains = {};  // speed (forward scale) per inch of distance to the point
  PidGains headingGains = {};
  ExitWindows exitWindows = MOVE_EXIT_WINDOWS;
  bool useCurrentForStall = false;
};

class HolonomicMoveToPoint : public MotionAlgorithm {
 public:
  explicit HolonomicMoveToPoint(const HolonomicMoveToPointSettings& settings);

  // Holds options.headingDegrees if set, otherwise the heading at start(). forwards and
  // direction don't apply: the robot never turns to face the point.
  void configure(const Point& target, const MotionOptions& options);
  void setSettings(const HolonomicMoveToPointSettings& settings);

  void start(const RobotState& state) override;
  MotionStep step(const RobotState& state, const MotionTick& tick,
                  DrivePipeline& pipeline) override;

  double distanceToTargetInches(const Pose& pose) const;
  double headingErrorDegrees(const Pose& pose) const;

 private:
  HolonomicMoveToPointSettings m_settings;
  MotionOptions m_options;
  Point m_target;
  double m_headingDegrees = 0.0;
  Pose m_lastPose;
  double m_travelledInches = 0.0;
  PidController m_translationPid;
  PidController m_headingPid;
  ExitConditions m_exits;
};

// The forward and strafe channels for moving at speed (on the forward 127 scale) along a field
// direction (a unit vector, or zero to stop). The strafe channel is rescaled by
// maxForward / maxStrafe so the physical path, not just the command, points that way.
struct TranslationChannels {
  ChannelRequest forward;
  ChannelRequest strafe;
};
TranslationChannels translationToward(const Pose& pose, const Point& fieldDirection, double speed,
                                      const DriveKinematics& kinematics, double maxSpeed);

}  // namespace cardLib
