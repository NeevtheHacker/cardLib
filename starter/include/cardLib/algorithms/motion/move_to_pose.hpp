// move_to_pose.hpp
// Why: moveToPose(x, y, theta): drive to a point, then turn in place once slowed (SPEC 8).
// Owns: the two phases (a MoveToPoint and a Turn) and the hand-off between them.
#pragma once

#include "cardLib/algorithms/interfaces/motion_algorithm.hpp"
#include "cardLib/algorithms/math/pose.hpp"
#include "cardLib/algorithms/motion/motion_options.hpp"
#include "cardLib/algorithms/motion/move_to_point.hpp"
#include "cardLib/algorithms/motion/turn.hpp"

namespace cardLib {

struct MoveToPoseSettings {
  MoveToPointSettings drive = {};
  TurnSettings turn = {};
  double handoffRadiusInches = 1.0;          // (tune)
  double handoffSpeedInchesPerSecond = 3.0;  // (tune)
};

class MoveToPose : public MotionAlgorithm {
 public:
  enum class Phase { Drive, Turn };

  explicit MoveToPose(const MoveToPoseSettings& settings);

  // minSpeed, earlyExit and direction apply to the final turn; the drive phase must slow down
  // to hand off, so it never uses them.
  void configure(const Pose& target, const MotionOptions& options);
  void setSettings(const MoveToPoseSettings& settings);

  void start(const RobotState& state) override;
  MotionStep step(const RobotState& state, const MotionTick& tick,
                  DrivePipeline& pipeline) override;

  Phase phase() const { return m_phase; }

 private:
  MotionStep stepDrive(const RobotState& state, const MotionTick& tick, DrivePipeline& pipeline);
  bool readyToTurn(const RobotState& state) const;
  MotionStep beginTurn(const RobotState& state, const MotionTick& tick, DrivePipeline& pipeline);

  MoveToPoseSettings m_settings;
  MotionOptions m_options;
  Pose m_target;
  Phase m_phase = Phase::Drive;
  double m_driveProgressInches = 0.0;
  MoveToPoint m_drive;
  Turn m_turn;
};

}  // namespace cardLib
