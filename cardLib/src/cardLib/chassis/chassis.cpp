// chassis.cpp
// Why: implements the Chassis API and the SPEC section 3 startup validation.
// Owns: nothing beyond Chassis' own members.
#include "cardLib/chassis/chassis.hpp"

#include <algorithm>
#include <cmath>

#include "cardLib/algorithms/control/drive_pipeline.hpp"
#include "cardLib/algorithms/control/field_centric.hpp"
#include "cardLib/hardware/distance_sensor.hpp"
#include "cardLib/hardware/gps_sensor.hpp"
#include "cardLib/hardware/imu.hpp"
#include "pros/misc.h"

namespace cardLib {

namespace {
constexpr std::uint32_t IMU_POLL_MS = 10;
constexpr std::uint32_t IMU_STATUS_SETTLE_MS = 50;  // calibration status takes a moment to show
constexpr std::uint32_t IMU_DRIFT_CHECK_MS = 1000;
}  // namespace

Chassis::Chassis(const ChassisConfig& config, PoseEstimator* estimator)
    : m_config(config),
      m_drive(resolveDrive(m_config)),
      m_defaultOdometry(makeArcOdometryConfig(m_config)),
      m_estimator(estimator != nullptr ? estimator : &m_defaultOdometry),
      m_odometry(m_config, *m_estimator, m_telemetry),
      m_motion(m_config, m_odometry, m_history, m_telemetry),
      m_driver(m_config.limits.throttleCurve, m_config.limits.turnCurve),
      m_gains(m_config.gains),
      m_driverBrake(m_config.limits.driverBrake) {}

// --- startup ----------------------------------------------------------------------------------

const StartupReport& Chassis::initialize() {
  m_report = {};
  if (!m_tasksStarted) {
    // Rebuilt here, not only in the constructor: the chassis may be a global built before
    // the sensor configs and motor groups it points to.
    const Pose pose = m_defaultOdometry.state().pose;
    m_defaultOdometry = ArcOdometry(makeArcOdometryConfig(m_config));
    m_defaultOdometry.reset(pose);
    m_drive = resolveDrive(m_config);
    m_kinematics = makeDriveKinematics(m_drive);
    m_motion.setDrive(m_drive, m_kinematics);
  }
  PortRegistry registry;
  registerDevices(registry);
  m_report.portIssueCount =
      std::min(registry.validate(m_report.portIssues), MAX_STARTUP_PORT_ISSUES);
  initializeDevices();
  m_report.noDrive = m_drive.moduleCount == 0;
  for (std::size_t i = 0; i < m_drive.moduleCount; ++i) {
    m_report.modulesWithoutMotors += m_drive.modules[i].motors == nullptr ? 1 : 0;
  }
  checkHeadingSource();
  calibrateImu();
  checkBattery();
  m_report.state = decideReadyState();
  m_report.validated = true;
  m_motion.setReadyState(m_report.state);

  if (!m_tasksStarted) {
    m_odometry.start();
    m_motion.start();
    m_tasksStarted = true;
  }
  return m_report;
}

const StartupReport& Chassis::calibrate() {
  m_report.imuCalibrationFailed = false;
  m_report.imuOrientationMismatch = false;
  m_report.imuDriftTooHigh = false;
  calibrateImu();
  m_report.state = decideReadyState();
  m_motion.setReadyState(m_report.state);
  return m_report;
}

bool Chassis::setCalibration(const Calibration& calibration) {
  if (m_tasksStarted) {
    return false;
  }
  m_config.calibration = calibration;
  m_defaultOdometry = ArcOdometry(makeArcOdometryConfig(m_config));
  return true;
}

void Chassis::registerDevices(PortRegistry& registry) const {
  const SensorConfig& sensors = m_config.sensors;
  const bool tankShorthand = m_config.drive.moduleCount == 0;
  for (std::size_t i = 0; i < m_drive.moduleCount; ++i) {
    const MotorGroup* motors = m_drive.modules[i].motors;
    if (motors == nullptr) {
      continue;
    }
    const char* role = motors->role();
    if (tankShorthand) {
      role = motors == m_config.drive.left ? "left drive" : "right drive";
    }
    motors->registerPorts(registry, role, Requirement::Required);
  }
  const std::array<std::pair<const TrackingWheelConfig*, const char*>, 3> wheels{{
      {sensors.verticalWheel1, "vertical wheel 1"},
      {sensors.verticalWheel2, "vertical wheel 2"},
      {sensors.horizontalWheel, "horizontal wheel"},
  }};
  for (const auto& [wheel, role] : wheels) {
    if (wheel != nullptr && wheel->sensor != nullptr) {
      wheel->sensor->registerPorts(registry, role, Requirement::Required);
    }
  }
  if (sensors.imu != nullptr && sensors.imu->sensor != nullptr) {
    registry.add(sensors.imu->sensor->port(), DeviceKind::Imu, "IMU", Requirement::Required);
  }
  for (const DistanceSensorConfig* distance : sensors.distanceSensors) {
    if (distance != nullptr && distance->sensor != nullptr) {
      registry.add(distance->sensor->port(), DeviceKind::Distance, "distance sensor",
                   Requirement::Optional);
    }
  }
  if (sensors.gps != nullptr && sensors.gps->sensor != nullptr) {
    registry.add(sensors.gps->sensor->port(), DeviceKind::Gps, "GPS", Requirement::Optional);
  }
}

void Chassis::initializeDevices() {
  const SensorConfig& sensors = m_config.sensors;
  for (std::size_t i = 0; i < m_drive.moduleCount; ++i) {
    if (m_drive.modules[i].motors != nullptr) {
      m_drive.modules[i].motors->initialize();
    }
  }
  for (const TrackingWheelConfig* wheel :
       {sensors.verticalWheel1, sensors.verticalWheel2, sensors.horizontalWheel}) {
    if (wheel != nullptr && wheel->sensor != nullptr) {
      wheel->sensor->initialize();
    }
  }
}

// At least one heading source: an IMU, or two parallel tracking wheels at different x.
void Chassis::checkHeadingSource() {
  const SensorConfig& sensors = m_config.sensors;
  const bool hasImu = sensors.imu != nullptr && sensors.imu->sensor != nullptr;
  const bool hasWheelPair = sensors.verticalWheel1 != nullptr &&
                            sensors.verticalWheel2 != nullptr &&
                            sensors.verticalWheel1->xInches != sensors.verticalWheel2->xInches;
  m_report.noHeadingSource = !hasImu && !hasWheelPair;
}

// Calibrates, checks the mounting the IMU detects, then a 1 s still drift check.
void Chassis::calibrateImu() {
  const ImuConfig* config = m_config.sensors.imu;
  if (config == nullptr || config->sensor == nullptr) {
    return;
  }
  Imu& imu = *config->sensor;
  if (!imu.startCalibration()) {
    m_report.imuCalibrationFailed = true;
    return;
  }
  pros::delay(IMU_STATUS_SETTLE_MS);
  std::uint32_t waitedMs = IMU_STATUS_SETTLE_MS;
  while (imu.isCalibrating() && waitedMs < m_config.limits.imuCalibrationTimeoutMs) {
    pros::delay(IMU_POLL_MS);
    waitedMs += IMU_POLL_MS;
  }
  if (imu.isCalibrating()) {
    m_report.imuCalibrationFailed = true;
    return;
  }

  m_report.imuDetectedOrientation = imu.physicalOrientation();
  m_report.imuOrientationMismatch = !m_report.imuDetectedOrientation.has_value() ||
                                    *m_report.imuDetectedOrientation != config->orientation;

  const std::optional<double> before = imu.readRotationDegrees();
  pros::delay(IMU_DRIFT_CHECK_MS);
  const std::optional<double> after = imu.readRotationDegrees();
  if (!before.has_value() || !after.has_value()) {
    m_report.imuCalibrationFailed = true;
    return;
  }
  m_report.imuDriftDegrees = std::abs(*after - *before);
  m_report.imuDriftTooHigh = m_report.imuDriftDegrees > m_config.limits.imuDriftLimitDegrees;
}

void Chassis::checkBattery() {
  m_report.batteryPercent = pros::c::battery_get_capacity();
  m_report.batteryLow = m_report.batteryPercent < m_config.limits.batteryWarningPercent;
}

ReadyState Chassis::decideReadyState() const {
  bool requiredPortFailed = false;
  for (std::size_t i = 0; i < m_report.portIssueCount; ++i) {
    requiredPortFailed =
        requiredPortFailed || m_report.portIssues[i].requirement == Requirement::Required;
  }
  if (requiredPortFailed || m_report.noDrive || m_report.modulesWithoutMotors > 0 ||
      m_report.noHeadingSource || m_report.imuOrientationMismatch) {
    return ReadyState::SensorFault;
  }
  if (m_report.imuCalibrationFailed || m_report.imuDriftTooHigh) {
    return ReadyState::NotCalibrated;
  }
  return ReadyState::Ready;
}

// --- pose ---------------------------------------------------------------------------------------

bool Chassis::setPose(double xInches, double yInches, double headingDegrees) {
  return m_odometry.requestReset({xInches, yInches, headingDegrees});
}

Pose Chassis::getPose() { return m_odometry.snapshot().state.pose; }

RobotState Chassis::getState() { return m_odometry.snapshot().state; }

OdometryHealth Chassis::odomHealth() { return m_odometry.snapshot().state.health; }

ResetResult Chassis::resetFromWall(std::size_t sensorIndex, Wall wall,
                                   const WallResetOptions& options) {
  if (!m_tasksStarted) {
    return ResetResult::NotRunning;
  }
  const DistanceSensorConfig* sensor =
      sensorIndex < MAX_DISTANCE_SENSORS ? m_config.sensors.distanceSensors[sensorIndex] : nullptr;
  if (sensor == nullptr || sensor->sensor == nullptr) {
    return ResetResult::NoSensor;
  }
  const OdometrySnapshot odometry = m_odometry.snapshot();
  const PoseResetDecision decision =
      wallReset(odometry.state, {sensor->xInches, sensor->yInches, sensor->headingDegrees},
                odometry.distances[sensorIndex], wall, m_config.field, options);
  const ResetResult result = applyReset(decision);
  if (result == ResetResult::Applied) {
    const bool alongX = wall == Wall::Left || wall == Wall::Right;
    m_telemetry.push({pros::millis(), TelemetryChannel::WallReset,
                      alongX ? decision.pose.xInches : decision.pose.yInches,
                      alongX ? odometry.state.pose.xInches : odometry.state.pose.yInches,
                      alongX ? 0.0 : 1.0});
  }
  return result;
}

ResetResult Chassis::resetFromGps(const GpsResetOptions& options) {
  if (!m_tasksStarted) {
    return ResetResult::NotRunning;
  }
  if (m_config.sensors.gps == nullptr || m_config.sensors.gps->sensor == nullptr) {
    return ResetResult::NoSensor;
  }
  const OdometrySnapshot odometry = m_odometry.snapshot();
  const PoseResetDecision decision = gpsReset(odometry.state, odometry.gps, options);
  const ResetResult result = applyReset(decision);
  if (result == ResetResult::Applied) {
    const Pose& before = odometry.state.pose;
    m_telemetry.push({pros::millis(), TelemetryChannel::GpsReset,
                      decision.pose.xInches - before.xInches,
                      decision.pose.yInches - before.yInches,
                      decision.pose.headingDegrees - before.headingDegrees});
  }
  return result;
}

// Hands an accepted correction to the odometry task, the only writer of the pose.
ResetResult Chassis::applyReset(const PoseResetDecision& decision) {
  if (decision.result != ResetResult::Applied) {
    return decision.result;
  }
  return m_odometry.requestReset(decision.pose) ? ResetResult::Applied : ResetResult::NotRunning;
}

// --- motions ------------------------------------------------------------------------------------

MotionResult Chassis::move(double distanceInches, const MotionOptions& options) {
  return runMotion({.kind = MotionKind::Move, .distanceInches = distanceInches,
                    .options = options});
}

MotionResult Chassis::turn(double headingDegrees, const MotionOptions& options) {
  return runMotion({.kind = MotionKind::Turn, .target = {0.0, 0.0, headingDegrees},
                    .options = options});
}

MotionResult Chassis::turnToPoint(double xInches, double yInches, const MotionOptions& options) {
  return runMotion({.kind = MotionKind::TurnToPoint, .target = {xInches, yInches, 0.0},
                    .options = options});
}

MotionResult Chassis::pivotTurn(double headingDegrees, Side lockedSide,
                                const MotionOptions& options) {
  return runMotion({.kind = MotionKind::PivotTurn,
                    .target = {0.0, 0.0, headingDegrees},
                    .lockedSide = lockedSide,
                    .options = options});
}

MotionResult Chassis::moveToPoint(double xInches, double yInches, const MotionOptions& options) {
  return runMotion({.kind = MotionKind::MoveToPoint, .target = {xInches, yInches, 0.0},
                    .options = options});
}

MotionResult Chassis::moveToPose(double xInches, double yInches, double headingDegrees,
                                 const MotionOptions& options) {
  return runMotion({.kind = MotionKind::MoveToPose,
                    .target = {xInches, yInches, headingDegrees},
                    .options = options});
}

MotionResult Chassis::strafe(double distanceInches, const MotionOptions& options) {
  return runMotion({.kind = MotionKind::Strafe, .distanceInches = distanceInches,
                    .options = options});
}

MotionResult Chassis::pivotTurn(double headingDegrees, const LocalOffset& pivotPoint,
                                const MotionOptions& options) {
  return runMotion({.kind = MotionKind::PivotTurn,
                    .target = {0.0, 0.0, headingDegrees},
                    .pivotAboutPoint = true,
                    .pivotPoint = pivotPoint,
                    .options = options});
}

MotionResult Chassis::runMotion(MotionCommand command) {
  if (!m_tasksStarted) {
    return MotionResult::NotCalibrated;
  }
  if (!supports(command)) {
    return refuseUnsupported(command);
  }
  command.options.timeoutMs = command.options.timeoutMs.value_or(m_config.limits.defaultTimeoutMs);
  command.options.allowDegraded = command.options.allowDegraded || m_allowDegraded.load();
  const std::uint32_t id = m_motion.post(command);
  m_lastMotionId.store(id);
  if (command.options.async) {
    return MotionResult::Started;
  }
  return m_motion.waitUntilDone(id);
}

// Strafing needs a wheel that pushes sideways; a pivot point off a tank's axle line needs
// sideways motion too (SPEC 20.9, 20.10).
bool Chassis::supports(const MotionCommand& command) const {
  if (command.kind == MotionKind::Strafe) {
    return m_kinematics.canStrafe();
  }
  if (command.kind == MotionKind::PivotTurn && command.pivotAboutPoint) {
    return m_kinematics.canPivotAbout(command.pivotPoint);
  }
  return true;
}

// The motion never starts, but the black box still records it (SPEC section 9).
MotionResult Chassis::refuseUnsupported(const MotionCommand& command) {
  const Pose pose = getPose();
  m_history.add({command.kind, command.target, pose, pose, MotionResult::Unsupported,
                 pros::millis(), 0});
  return MotionResult::Unsupported;
}

bool Chassis::waitUntil(double progress) {
  return m_tasksStarted && m_motion.waitUntil(m_lastMotionId.load(), progress);
}

MotionResult Chassis::waitUntilDone() {
  if (!m_tasksStarted) {
    return MotionResult::NotCalibrated;
  }
  return m_motion.waitUntilDone(m_lastMotionId.load());
}

bool Chassis::isMoving() { return m_motion.status().moving; }

void Chassis::cancel() { m_motion.cancel(); }

// --- driver control -----------------------------------------------------------------------------

void Chassis::tank(double leftStick, double rightStick) {
  m_motion.drive(m_driver.tank(leftStick, rightStick));
}

void Chassis::arcade(double forwardStick, double turnStick) {
  m_motion.drive(m_driver.arcade(forwardStick, turnStick));
}

void Chassis::holonomic(double forwardStick, double strafeStick, double turnStick,
                        const HolonomicOptions& options) {
  const bool fieldCentric =
      options.fieldCentric.value_or(m_driveMode.load() == DriveMode::FieldCentric);
  if (fieldCentric && m_tasksStarted && !m_kinematics.canStrafe()) {
    m_fieldCentricIgnored.store(true);
  } else if (fieldCentric) {
    const OdometrySnapshot odometry = m_odometry.snapshot();
    if (fieldHeadingAvailable(odometry)) {
      const double heading = odometry.state.pose.headingDegrees - m_fieldForwardDegrees.load();
      const StickVector robot = fieldCentricSticks(forwardStick, strafeStick, heading);
      forwardStick = robot.forward;
      strafeStick = robot.strafe;
    }
  }
  m_motion.drive(m_driver.holonomic(forwardStick, strafeStick, turnStick));
}

void Chassis::setDriveMode(DriveMode mode) {
  m_driveMode.store(mode);
  if (mode == DriveMode::FieldCentric && m_tasksStarted && !m_kinematics.canStrafe()) {
    m_fieldCentricIgnored.store(true);
  }
}

void Chassis::resetFieldHeading() { m_fieldForwardDegrees.store(getPose().headingDegrees); }

// Field-centric needs a heading it can trust: a configured IMU that hasn't faulted.
bool Chassis::fieldHeadingAvailable(const OdometrySnapshot& odometry) const {
  const ImuConfig* imu = m_config.sensors.imu;
  return imu != nullptr && imu->sensor != nullptr && !odometry.imuFaulted;
}

void Chassis::driveMillivolts(std::int32_t leftMillivolts, std::int32_t rightMillivolts) {
  m_motion.drive(
      {.direct = {.leftMillivolts = leftMillivolts, .rightMillivolts = rightMillivolts}});
}

void Chassis::driveMillivolts(std::span<const std::int32_t> wheelMillivolts) {
  DriverCommand command;
  command.direct.wheelCount = m_drive.moduleCount;
  for (std::size_t i = 0; i < m_drive.moduleCount && i < wheelMillivolts.size(); ++i) {
    command.direct.wheels[i].millivolts = wheelMillivolts[i];
  }
  summarizeSides(command.direct, m_kinematics);
  m_motion.drive(command);
}

void Chassis::setDriverBrake(BrakeMode mode) {
  m_driverBrake.store(mode);
  m_motion.setDriverBrake(mode);
}

// --- tuning -------------------------------------------------------------------------------------

void Chassis::setGains(const Gains& gains) {
  m_gainsMutex.take();
  m_gains = gains;
  m_gainsMutex.give();
  m_motion.setGains(gains);
}

Gains Chassis::gains() {
  m_gainsMutex.take();
  const Gains copy = m_gains;
  m_gainsMutex.give();
  return copy;
}

}  // namespace cardLib
