// sim_robot.hpp
// Why: runs a real motion against the simulator the way the motion and odometry tasks will:
//      read state, step the motion, write the drive, advance 10 ms, read the sensors through
//      plausibility checks, update odometry.
// Owns: the simulated drive, the robot's own estimator, the encoder checks and the shared
//       drive pipeline. Test-only.
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "cardLib/algorithms/control/drive_pipeline.hpp"
#include "cardLib/algorithms/interfaces/motion_algorithm.hpp"
#include "cardLib/algorithms/localization/arc_odometry.hpp"
#include "cardLib/algorithms/localization/plausibility_check.hpp"
#include "drive_sim.hpp"

namespace cardLib::sim {

struct TraceSample {
  std::uint32_t elapsedMs = 0;
  Pose truePose = {};
  double linearSpeedInchesPerSecond = 0.0;
  OdometryHealth health = OdometryHealth::Ok;
};

struct SimRun {
  MotionResult result = MotionResult::TimedOut;
  bool brake = true;
  bool finished = false;
  std::uint32_t elapsedMs = 0;
  std::vector<TraceSample> trace;
};

class SimRobot {
 public:
  SimRobot(const DriveSimConfig& config, const SlewRates& linearRates,
           const SlewRates& angularRates, const Pose& start = {});

  // Runs one motion until it exits or maxMs passes. A braking exit stops the drive and resets
  // the slew; a non-braking exit (earlyExit) coasts one loop and keeps the slew for chaining.
  SimRun run(MotionAlgorithm& motion, std::uint32_t maxMs = 5000);

  // Leaves the drive at 0 V for a while, e.g. to watch a braked robot settle.
  void idle(std::uint32_t durationMs);

  // During the next run(), the left encoder read fails for this many loops in a row, starting
  // this far into the motion.
  void failLeftEncoderReads(std::uint32_t atElapsedMs, std::uint32_t loops);

  const RobotState& state() const { return m_state; }
  DriveSim& drive() { return m_drive; }

 private:
  void tick(const DriveCommand& command, std::uint32_t elapsedMs);
  SensorFrame readSensors(std::uint32_t elapsedMs);

  DriveSim m_drive;
  ArcOdometry m_estimator;
  PlausibilityCheck m_leftCheck;
  PlausibilityCheck m_rightCheck;
  DrivePipeline m_pipeline;
  RobotState m_state;
  std::uint32_t m_failStartMs = 0;
  std::uint32_t m_failLoops = 0;
};

}  // namespace cardLib::sim
