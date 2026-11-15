// chassis.hpp
// Why: the API auton and driver code calls: motions, waits, pose, driver control (SPEC 8 to 10).
// Owns: the odometry and motion tasks, the default estimator, and the startup report.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "cardLib/algorithms/control/drive_kinematics.hpp"
#include "cardLib/algorithms/interfaces/pose_estimator.hpp"
#include "cardLib/algorithms/localization/arc_odometry.hpp"
#include "cardLib/algorithms/motion/motion_options.hpp"
#include "cardLib/algorithms/motion/motion_result.hpp"
#include "cardLib/chassis/chassis_config.hpp"
#include "cardLib/chassis/driver_control.hpp"
#include "cardLib/chassis/motion_command.hpp"
#include "cardLib/chassis/motion_history.hpp"
#include "cardLib/chassis/motion_task.hpp"
#include "cardLib/chassis/odometry_task.hpp"
#include "cardLib/chassis/telemetry_buffer.hpp"
#include "cardLib/hardware/port_registry.hpp"
#include "pros/rtos.hpp"

namespace cardLib {

constexpr std::size_t MAX_STARTUP_PORT_ISSUES = 16;

// What startup validation found (SPEC section 3). The display shows it; motions obey state.
struct StartupReport {
  bool validated = false;  // initialize() has run
  ReadyState state = ReadyState::NotCalibrated;
  std::array<PortIssue, MAX_STARTUP_PORT_ISSUES> portIssues = {};
  std::size_t portIssueCount = 0;
  bool noHeadingSource = false;
  bool imuCalibrationFailed = false;
  bool imuOrientationMismatch = false;
  std::optional<ImuOrientation> imuDetectedOrientation = std::nullopt;
  double imuDriftDegrees = 0.0;
  bool imuDriftTooHigh = false;
  bool batteryLow = false;  // a warning only; it doesn't block motions
  double batteryPercent = 0.0;
};

class Chassis {
 public:
  // Without an estimator, the chassis uses its own arc odometry built from the config.
  // Safe as a global: the constructor only stores the config and touches no device.
  explicit Chassis(const ChassisConfig& config, PoseEstimator* estimator = nullptr);
  Chassis(const Chassis&) = delete;
  Chassis& operator=(const Chassis&) = delete;

  // Call once from initialize(), robot still: validates devices, calibrates the IMU (a few
  // seconds), then starts the odometry and motion tasks.
  const StartupReport& initialize();

  // Re-runs IMU calibration and the drift check. The robot must be still.
  const StartupReport& calibrate();

  // Replaces the calibration values, e.g. after loading the SD file. Only before
  // initialize(); false once the tasks are running.
  bool setCalibration(const Calibration& calibration);

  // Pose. setPose is applied by the odometry task within one loop; false if it timed out.
  bool setPose(double xInches, double yInches, double headingDegrees);
  Pose getPose();
  RobotState getState();
  OdometryHealth odomHealth();

  // Motions. Each blocks until done and returns its result, unless options.async is set, in
  // which case it returns MotionResult::Started at once.
  MotionResult move(double distanceInches, const MotionOptions& options = {});
  MotionResult turn(double headingDegrees, const MotionOptions& options = {});
  MotionResult turnToPoint(double xInches, double yInches, const MotionOptions& options = {});
  MotionResult pivotTurn(double headingDegrees, Side lockedSide,
                         const MotionOptions& options = {});
  MotionResult moveToPoint(double xInches, double yInches, const MotionOptions& options = {});
  MotionResult moveToPose(double xInches, double yInches, double headingDegrees,
                          const MotionOptions& options = {});

  // Motion sync, for the last motion started. waitUntil counts inches for moves and degrees
  // for turns; false if the motion ended first.
  bool waitUntil(double progress);
  MotionResult waitUntilDone();
  bool isMoving();
  void cancel();

  // Driver control, -127 to 127. Either one cancels a running motion.
  void tank(double leftStick, double rightStick);
  void arcade(double forwardStick, double turnStick);
  // Raw side voltages through the motion task, no curves. For test and calibration routines.
  // Every left module gets the left value and every right module the right one.
  void driveMillivolts(std::int32_t leftMillivolts, std::int32_t rightMillivolts);
  // Raw voltage per wheel module, in the drive's module order; missing entries get 0.
  void driveMillivolts(std::span<const std::int32_t> wheelMillivolts);
  void setDriverBrake(BrakeMode mode);
  BrakeMode driverBrake() const { return m_driverBrake.load(); }

  // Run motions even on degraded odometry (SPEC section 4). Off by default.
  void setAllowDegraded(bool allow) { m_allowDegraded.store(allow); }

  // Tuning: takes effect at the next motion loop.
  void setGains(const Gains& gains);
  Gains gains();

  const StartupReport& startupReport() const { return m_report; }
  const ChassisConfig& config() const { return m_config; }
  // The drive as wheel modules, and its kinematics (built in initialize()).
  const DriveConfig& drive() const { return m_drive; }
  const DriveKinematics& driveKinematics() const { return m_kinematics; }
  OdometrySnapshot odometrySnapshot() { return m_odometry.snapshot(); }
  MotionStatus motionStatus() { return m_motion.status(); }
  MotionHistory& motionHistory() { return m_history; }
  TelemetryBuffer& telemetry() { return m_telemetry; }

 private:
  MotionResult runMotion(MotionCommand command);
  void registerDevices(PortRegistry& registry) const;
  void initializeDevices();
  void checkHeadingSource();
  void calibrateImu();
  void checkBattery();
  ReadyState decideReadyState() const;

  ChassisConfig m_config;
  DriveConfig m_drive;
  DriveKinematics m_kinematics;
  ArcOdometry m_defaultOdometry;
  PoseEstimator* m_estimator;
  TelemetryBuffer m_telemetry;
  MotionHistory m_history;
  OdometryTask m_odometry;
  MotionTask m_motion;
  DriverControl m_driver;
  StartupReport m_report;
  pros::Mutex m_gainsMutex;
  Gains m_gains;
  std::atomic<std::uint32_t> m_lastMotionId{0};
  std::atomic<bool> m_allowDegraded{false};
  std::atomic<BrakeMode> m_driverBrake;
  bool m_tasksStarted = false;
};

}  // namespace cardLib
