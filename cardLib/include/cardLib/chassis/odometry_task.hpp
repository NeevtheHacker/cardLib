// odometry_task.hpp
// Why: keeps the robot's field position up to date every 10 ms.
// Owns: the pose (only writer). Reads: IMU, tracking wheels, drive encoders, distance sensors.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>

#include "cardLib/algorithms/interfaces/pose_estimator.hpp"
#include "cardLib/algorithms/interfaces/robot_state.hpp"
#include "cardLib/algorithms/localization/plausibility_check.hpp"
#include "cardLib/algorithms/localization/sensor_frame.hpp"
#include "cardLib/chassis/chassis_config.hpp"
#include "cardLib/chassis/telemetry_buffer.hpp"
#include "pros/rtos.hpp"

namespace cardLib {

// Everything odometry publishes each loop: the state plus which sensors are working.
struct OdometrySnapshot {
  RobotState state = {};
  std::array<bool, MAX_VERTICAL_WHEELS> verticalWheelFaulted = {};
  bool horizontalWheelFaulted = false;
  bool imuAnswering = true;
  bool leftDriveFaulted = false;
  bool rightDriveFaulted = false;
  bool driveMotorMissing = false;  // a drive motor stopped answering: motions report SensorFault
  std::array<DistanceReading, MAX_DISTANCE_SENSORS> distances = {};
  std::uint32_t loopOverruns = 0;
};

class OdometryTask {
 public:
  static constexpr std::uint32_t PRIORITY = 10;

  OdometryTask(const ChassisConfig& config, PoseEstimator& estimator,
               TelemetryBuffer& telemetry);

  // Starts the 10 ms task. Call once, after the IMU has calibrated.
  void start();
  bool running() const { return m_task != nullptr; }

  // A copy of the latest snapshot. The lock is held only for the copy.
  OdometrySnapshot snapshot();

  // Asks the task to set the pose and re-baseline every sensor at the start of its next loop,
  // then waits for it. False if it wasn't applied within timeoutMs.
  bool requestReset(const Pose& pose, std::uint32_t timeoutMs = 50);

 private:
  static void taskEntry(void* self);
  void run();
  void applyPendingReset();
  void rebaselineSensors();
  SensorFrame buildSensorFrame(double dtSeconds);
  void readTrackingWheels(SensorFrame& frame, double dtSeconds);
  void readDrive(SensorFrame& frame, double dtSeconds);
  void readImu(SensorFrame& frame);
  void readDistances(SensorFrame& frame);
  void publish(const RobotState& state, const SensorFrame& frame);
  std::optional<double> verticalWheelInches(std::size_t index);
  std::optional<double> horizontalWheelInches();
  std::optional<double> driveInches(MotorGroup* group);

  const ChassisConfig& m_config;
  PoseEstimator& m_estimator;
  TelemetryBuffer& m_telemetry;
  std::array<const TrackingWheelConfig*, MAX_VERTICAL_WHEELS> m_verticalWheels = {};
  std::array<PlausibilityCheck, MAX_VERTICAL_WHEELS> m_verticalChecks;
  PlausibilityCheck m_horizontalCheck;
  PlausibilityCheck m_leftDriveCheck;
  PlausibilityCheck m_rightDriveCheck;
  pros::MutexVar<OdometrySnapshot> m_shared;
  pros::Mutex m_resetMutex;
  bool m_resetPending = false;
  Pose m_resetPose = {};
  std::atomic<std::uint32_t> m_resetsApplied{0};
  std::uint32_t m_loopOverruns = 0;
  std::uint32_t m_loopCount = 0;
  pros::task_t m_task = nullptr;
};

}  // namespace cardLib
