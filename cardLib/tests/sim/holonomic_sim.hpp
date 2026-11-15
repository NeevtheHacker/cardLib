// holonomic_sim.hpp
// Why: a laptop stand-in for any wheel-module drive (SPEC 20), so holonomic motions can be run
//      whole: each wheel chases its commanded speed with a lag, the body moves by the
//      least-squares fit of the wheels, and the robot reads tracking wheels, an IMU and its
//      drive encoders through its own odometry.
// Owns: each wheel's speed, the true pose, the robot's estimator and the shared pipeline.
//       Test-only.
#pragma once

#include <array>
#include <cstdint>

#include "cardLib/algorithms/control/drive_kinematics.hpp"
#include "cardLib/algorithms/control/drive_pipeline.hpp"
#include "cardLib/algorithms/interfaces/motion_algorithm.hpp"
#include "cardLib/algorithms/localization/arc_odometry.hpp"
#include "sim_robot.hpp"

namespace cardLib::sim {

struct HolonomicSimConfig {
  DriveKinematics kinematics = {};
  double timeConstantSeconds = 0.15;  // tau, as in DriveSim
  bool horizontalWheel = true;        // a horizontal tracking wheel at the center
  bool verticalWheel = true;          // a vertical tracking wheel at the center
};

class HolonomicSimRobot {
 public:
  HolonomicSimRobot(const HolonomicSimConfig& config, const SlewRates& linearRates,
                    const SlewRates& strafeRates, const SlewRates& angularRates,
                    const Pose& start = {});

  // Runs one motion until it exits or maxMs passes, like SimRobot::run.
  SimRun run(MotionAlgorithm& motion, std::uint32_t maxMs = 5000);
  void idle(std::uint32_t durationMs);

  // The true position of a robot-local point (e.g. a wheel's contact point).
  Point truePointOnRobot(const LocalOffset& local) const;

  const Pose& truePose() const { return m_truePose; }
  const RobotState& state() const { return m_state; }

 private:
  void tick(const DriveCommand& command);

  HolonomicSimConfig m_config;
  ArcOdometry m_estimator;
  DrivePipeline m_pipeline;
  RobotState m_state;
  Pose m_truePose;
  std::array<double, MAX_WHEEL_MODULES> m_wheelSpeeds = {};
};

}  // namespace cardLib::sim
