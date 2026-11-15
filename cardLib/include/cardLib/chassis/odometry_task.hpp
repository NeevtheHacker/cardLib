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

// The last good reading of each sensor as a running total, in inches with the current
// calibration (IMU raw and unscaled). Calibration routines and test modes use these, so no
// other task ever reads a sensor the odometry task owns.
struct SensorTotals {
  std::array<double, MAX_VERTICAL_WHEELS> verticalWheelInches = {};
  double horizontalWheelInches = 0.0;
  double leftDriveInches = 0.0;   // mean of the left modules (a tank's left side)
  double rightDriveInches = 0.0;  // mean of the right modules
  double imuRotationDegrees = 0.0;
  std::array<double, MAX_WHEEL_MODULES> driveWheelInches = {};  // each module, in drive order
};

// How many readings each sensor has had rejected or failed since start-up (SPEC section 4: a
// single bad reading is logged, not a fault).
struct SensorRejections {
  std::array<std::uint32_t, MAX_VERTICAL_WHEELS> verticalWheels = {};
  std::uint32_t horizontalWheel = 0;
  std::uint32_t leftDrive = 0;
  std::uint32_t rightDrive = 0;
  std::uint32_t imu = 0;
};

// Everything odometry publishes each loop: the state plus which sensors are working.
struct OdometrySnapshot {
  RobotState state = {};
  std::array<bool, MAX_VERTICAL_WHEELS> verticalWheelFaulted = {};
  bool horizontalWheelFaulted = false;
  bool imuAnswering = true;  // this loop's read worked
  bool imuFaulted = false;   // three failed reads in a row
  bool leftDriveFaulted = false;
  bool rightDriveFaulted = false;
  bool driveMotorMissing = false;  // a drive motor stopped answering: motions report SensorFault
  std::array<DistanceReading, MAX_DISTANCE_SENSORS> distances = {};
  GpsReading gps = {};      // the robot's center from the GPS, refreshed at 10 Hz
  bool gpsFaulted = false;  // three failed GPS reads in a row
  SensorTotals totals = {};
  SensorRejections rejections = {};
  std::uint32_t loopOverruns = 0;
};

class OdometryTask {
 public:
  static constexpr std::uint32_t PRIORITY = 10;

  OdometryTask(const ChassisConfig& config, PoseEstimator& estimator, TelemetryBuffer& telemetry);

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
  void combineSides(SensorFrame& frame, const std::array<WheelDelta, MAX_WHEEL_MODULES>& deltas);
  void readImu(SensorFrame& frame);
  void readDistances(SensorFrame& frame);
  void readGps();
  void publish(const RobotState& state, const SensorFrame& frame);
  SensorRejections rejectionCounts() const;
  std::optional<double> verticalWheelInches(std::size_t index);
  std::optional<double> horizontalWheelInches();
  std::optional<double> moduleInches(std::size_t module);
  bool moduleOnSide(std::size_t module, Side side) const;

  const ChassisConfig& m_config;
  DriveConfig m_drive;  // resolved: always wheel modules
  PoseEstimator& m_estimator;
  TelemetryBuffer& m_telemetry;
  std::array<const TrackingWheelConfig*, MAX_VERTICAL_WHEELS> m_verticalWheels = {};
  std::array<PlausibilityCheck, MAX_VERTICAL_WHEELS> m_verticalChecks;
  PlausibilityCheck m_horizontalCheck;
  std::array<PlausibilityCheck, MAX_WHEEL_MODULES> m_driveChecks;
  pros::MutexVar<OdometrySnapshot> m_shared;
  pros::Mutex m_resetMutex;
  bool m_resetPending = false;
  Pose m_resetPose = {};
  std::atomic<std::uint32_t> m_resetsApplied{0};
  SensorTotals m_totals = {};
  std::uint32_t m_imuFailuresInARow = 0;
  std::uint32_t m_imuRejections = 0;
  bool m_imuFaulted = false;
  std::uint32_t m_loopOverruns = 0;
  std::uint32_t m_loopCount = 0;
  std::uint32_t m_gpsLoop = 0;
  GpsReading m_gps = {};
  pros::task_t m_task = nullptr;
};

}  // namespace cardLib
