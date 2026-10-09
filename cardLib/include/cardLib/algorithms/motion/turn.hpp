// turn.hpp
// Why: turn(theta) and turnToPoint(x, y): tank turns in place (SPEC section 8).
// Owns: the turn PID, exit timers, the target and the remaining forced direction.
#pragma once

#include "cardLib/algorithms/control/exit_conditions.hpp"
#include "cardLib/algorithms/control/pid_controller.hpp"
#include "cardLib/algorithms/interfaces/motion_algorithm.hpp"
#include "cardLib/algorithms/math/pose.hpp"
#include "cardLib/algorithms/motion/motion_options.hpp"

namespace cardLib {

struct TurnSettings {
  PidGains turnGains = {};
  ExitWindows exitWindows = TURN_EXIT_WINDOWS;
  bool useCurrentForStall = false;
};

// Closer than this to a turnToPoint target, the heading to it is meaningless; keep the last one.
constexpr double TURN_TO_POINT_MIN_DISTANCE_INCHES = 0.1;

class Turn : public MotionAlgorithm {
 public:
  explicit Turn(const TurnSettings& settings);

  // turn(theta): face a fixed heading.
  void configureHeading(double headingDegrees, const MotionOptions& options);
  // turnToPoint(x, y): face a point, recomputed every loop; forwards = false faces away.
  void configurePoint(const Point& target, const MotionOptions& options);
  void setSettings(const TurnSettings& settings);

  void start(const RobotState& state) override;
  MotionStep step(const RobotState& state, const MotionTick& tick,
                  DrivePipeline& pipeline) override;

 private:
  enum class TargetKind { Heading, Point };

  void updateTargetHeading(const Pose& pose);

  TurnSettings m_settings;
  MotionOptions m_options;
  TargetKind m_kind = TargetKind::Heading;
  Point m_targetPoint;
  double m_targetHeadingDegrees = 0.0;
  double m_startHeadingDegrees = 0.0;
  TurnDirection m_direction = TurnDirection::Shortest;
  PidController m_pid;
  ExitConditions m_exits;
};

}  // namespace cardLib
