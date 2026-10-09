// odometry_task.cpp
// Why: the odometry loop: read every sensor once, build the SensorFrame, run the estimator,
//      publish (SPEC sections 2, 4 and 5).
// Owns: nothing beyond OdometryTask's own state.
#include "cardLib/chassis/odometry_task.hpp"

#include "cardLib/algorithms/localization/wheel_distance.hpp"
#include "cardLib/algorithms/math/loop_timing.hpp"
#include "cardLib/hardware/distance_sensor.hpp"
#include "cardLib/hardware/imu.hpp"

namespace cardLib {

namespace {
constexpr std::uint32_t POSE_TELEMETRY_EVERY_LOOPS = 10;  // 10 Hz
constexpr double MICROSECONDS_PER_SECOND = 1e6;
constexpr std::uint32_t RESET_POLL_MS = 2;
constexpr std::uint32_t IMU_FAILURES_TO_FAULT = 3;
}  // namespace

OdometryTask::OdometryTask(const ChassisConfig& config, PoseEstimator& estimator,
                           TelemetryBuffer& telemetry)
    : m_config(config),
      m_estimator(estimator),
      m_telemetry(telemetry),
      // The real speed limit is set in start(), once the motor groups exist. The chassis may
      // be a global built before them.
      m_verticalChecks{PlausibilityCheck(0.0), PlausibilityCheck(0.0)},
      m_horizontalCheck(0.0),
      m_leftDriveCheck(0.0),
      m_rightDriveCheck(0.0) {
  std::size_t count = 0;
  for (const TrackingWheelConfig* wheel :
       {config.sensors.verticalWheel1, config.sensors.verticalWheel2}) {
    if (wheel != nullptr) {
      m_verticalWheels[count++] = wheel;
    }
  }
}

void OdometryTask::start() {
  if (m_task != nullptr) {
    return;
  }
  const double maxSpeed = maxDriveSpeedInchesPerSecond(m_config);
  m_verticalChecks = {PlausibilityCheck(maxSpeed), PlausibilityCheck(maxSpeed)};
  m_horizontalCheck = PlausibilityCheck(maxSpeed);
  m_leftDriveCheck = PlausibilityCheck(maxSpeed);
  m_rightDriveCheck = PlausibilityCheck(maxSpeed);
  m_task = pros::c::task_create(&OdometryTask::taskEntry, this, PRIORITY,
                                TASK_STACK_DEPTH_DEFAULT, "cardLib odometry");
}

void OdometryTask::taskEntry(void* self) { static_cast<OdometryTask*>(self)->run(); }

void OdometryTask::run() {
  rebaselineSensors();
  std::uint32_t wakeTimeMs = pros::millis();
  std::uint64_t lastMicros = pros::micros();
  while (true) {
    applyPendingReset();
    const std::uint64_t nowMicros = pros::micros();
    const double dtSeconds = (nowMicros - lastMicros) / MICROSECONDS_PER_SECOND;
    lastMicros = nowMicros;
    if (isLoopOverrun(dtSeconds)) {
      ++m_loopOverruns;
    }
    const SensorFrame frame = buildSensorFrame(dtSeconds);
    publish(m_estimator.update(frame), frame);
    pros::Task::delay_until(&wakeTimeMs, LOOP_PERIOD_MS);
  }
}

OdometrySnapshot OdometryTask::snapshot() { return *m_shared.lock(); }

bool OdometryTask::requestReset(const Pose& pose, std::uint32_t timeoutMs) {
  const std::uint32_t before = m_resetsApplied.load();
  m_resetMutex.take();
  m_resetPose = pose;
  m_resetPending = true;
  m_resetMutex.give();
  if (!running()) {
    applyPendingReset();
    return true;
  }
  for (std::uint32_t waitedMs = 0; waitedMs <= timeoutMs; waitedMs += RESET_POLL_MS) {
    if (m_resetsApplied.load() != before) {
      return true;
    }
    pros::delay(RESET_POLL_MS);
  }
  return false;
}

void OdometryTask::applyPendingReset() {
  m_resetMutex.take();
  const bool pending = m_resetPending;
  const Pose pose = m_resetPose;
  m_resetPending = false;
  m_resetMutex.give();
  if (!pending) {
    return;
  }
  rebaselineSensors();
  m_estimator.reset(pose);
  m_shared.lock()->state.pose = pose;
  m_resetsApplied.fetch_add(1);
}

void OdometryTask::rebaselineSensors() {
  for (std::size_t i = 0; i < MAX_VERTICAL_WHEELS; ++i) {
    m_verticalChecks[i].rebaseline(verticalWheelInches(i));
  }
  m_horizontalCheck.rebaseline(horizontalWheelInches());
  m_leftDriveCheck.rebaseline(driveInches(m_config.drive.left));
  m_rightDriveCheck.rebaseline(driveInches(m_config.drive.right));
  m_imuFailuresInARow = 0;
  m_imuFaulted = false;
}

SensorFrame OdometryTask::buildSensorFrame(double dtSeconds) {
  SensorFrame frame;
  frame.dtSeconds = dtSeconds;
  readTrackingWheels(frame, dtSeconds);
  readDrive(frame, dtSeconds);
  readImu(frame);
  readDistances(frame);
  return frame;
}

namespace {

// Keeps the last good reading as the published raw total, and passes the reading on.
std::optional<double> remember(std::optional<double> reading, double& total) {
  if (reading.has_value()) {
    total = *reading;
  }
  return reading;
}

}  // namespace

void OdometryTask::readTrackingWheels(SensorFrame& frame, double dtSeconds) {
  for (std::size_t i = 0; i < MAX_VERTICAL_WHEELS; ++i) {
    if (m_verticalWheels[i] != nullptr) {
      const auto inches = remember(verticalWheelInches(i), m_totals.verticalWheelInches[i]);
      frame.verticalWheels[i] = m_verticalChecks[i].measure(inches, dtSeconds);
    }
  }
  if (m_config.sensors.horizontalWheel != nullptr) {
    const auto inches = remember(horizontalWheelInches(), m_totals.horizontalWheelInches);
    frame.horizontalWheel = m_horizontalCheck.measure(inches, dtSeconds);
  }
}

void OdometryTask::readDrive(SensorFrame& frame, double dtSeconds) {
  const auto left = remember(driveInches(m_config.drive.left), m_totals.leftDriveInches);
  const auto right = remember(driveInches(m_config.drive.right), m_totals.rightDriveInches);
  frame.leftDrive = m_leftDriveCheck.measure(left, dtSeconds);
  frame.rightDrive = m_rightDriveCheck.measure(right, dtSeconds);
}

void OdometryTask::readImu(SensorFrame& frame) {
  const ImuConfig* imu = m_config.sensors.imu;
  if (imu == nullptr || imu->sensor == nullptr) {
    return;
  }
  const std::optional<double> rotation =
      remember(imu->sensor->readRotationDegrees(), m_totals.imuRotationDegrees);
  frame.imuReadOk = rotation.has_value();
  if (frame.imuReadOk) {
    m_imuFailuresInARow = 0;
  } else {
    ++m_imuFailuresInARow;
    ++m_imuRejections;
  }
  // Three failed reads in a row fault the IMU until the next setPose (SPEC section 4).
  m_imuFaulted = m_imuFaulted || m_imuFailuresInARow >= IMU_FAILURES_TO_FAULT;
  frame.imuFaulted = m_imuFaulted;
  frame.imuRotationDegrees = rotation.value_or(0.0);
  frame.imuGyroRateDegreesPerSecond = imu->sensor->readGyroRateDegreesPerSecond().value_or(0.0);
}

void OdometryTask::readDistances(SensorFrame& frame) {
  for (std::size_t i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
    const DistanceSensorConfig* config = m_config.sensors.distanceSensors[i];
    if (config == nullptr || config->sensor == nullptr) {
      continue;
    }
    const std::optional<double> raw = config->sensor->readInches();
    if (!raw.has_value()) {
      continue;
    }
    frame.distances[i] = {config->gain * *raw + config->offsetInches,
                          config->sensor->readConfidence().value_or(0.0), true};
  }
}

std::optional<double> OdometryTask::verticalWheelInches(std::size_t index) {
  const TrackingWheelConfig* wheel = m_verticalWheels[index];
  if (wheel == nullptr || wheel->sensor == nullptr) {
    return std::nullopt;
  }
  const std::optional<double> degrees = wheel->sensor->readDegrees();
  if (!degrees.has_value()) {
    return std::nullopt;
  }
  const double diameter =
      m_config.calibration.verticalWheelDiameterInches[index].value_or(wheel->diameterInches);
  return encoderDegreesToInches(*degrees, diameter, wheel->gearRatio);
}

std::optional<double> OdometryTask::horizontalWheelInches() {
  const TrackingWheelConfig* wheel = m_config.sensors.horizontalWheel;
  if (wheel == nullptr || wheel->sensor == nullptr) {
    return std::nullopt;
  }
  const std::optional<double> degrees = wheel->sensor->readDegrees();
  if (!degrees.has_value()) {
    return std::nullopt;
  }
  const double diameter =
      m_config.calibration.horizontalWheelDiameterInches.value_or(wheel->diameterInches);
  return encoderDegreesToInches(*degrees, diameter, wheel->gearRatio);
}

std::optional<double> OdometryTask::driveInches(MotorGroup* group) {
  if (group == nullptr) {
    return std::nullopt;
  }
  const std::optional<double> degrees = group->readDegrees();
  if (!degrees.has_value()) {
    return std::nullopt;
  }
  return motorEncoderInches(*degrees, m_config.geometry.wheelDiameterInches,
                            m_config.geometry.externalRatio, m_config.calibration.driveScale);
}

void OdometryTask::publish(const RobotState& state, const SensorFrame& frame) {
  const MotorGroup* left = m_config.drive.left;
  const MotorGroup* right = m_config.drive.right;
  {
    auto shared = m_shared.lock();
    shared->state = state;
    for (std::size_t i = 0; i < MAX_VERTICAL_WHEELS; ++i) {
      shared->verticalWheelFaulted[i] = m_verticalChecks[i].faulted();
    }
    shared->horizontalWheelFaulted = m_horizontalCheck.faulted();
    shared->imuAnswering = frame.imuReadOk;
    shared->imuFaulted = frame.imuFaulted;
    shared->rejections = rejectionCounts();
    shared->leftDriveFaulted = m_leftDriveCheck.faulted();
    shared->rightDriveFaulted = m_rightDriveCheck.faulted();
    // A drive motor is missing only after three failed reads in a row, not on one glitch.
    shared->driveMotorMissing = (left != nullptr && left->anyMotorFaulted()) ||
                                (right != nullptr && right->anyMotorFaulted());
    shared->distances = frame.distances;
    shared->totals = m_totals;
    shared->loopOverruns = m_loopOverruns;
  }
  if ((m_config.limits.telemetryChannels & TELEMETRY_POSE) != 0 &&
      ++m_loopCount % POSE_TELEMETRY_EVERY_LOOPS == 0) {
    m_telemetry.push({pros::millis(), TelemetryChannel::Pose, state.pose.xInches,
                      state.pose.yInches, state.pose.headingDegrees});
  }
}

SensorRejections OdometryTask::rejectionCounts() const {
  SensorRejections counts;
  for (std::size_t i = 0; i < MAX_VERTICAL_WHEELS; ++i) {
    counts.verticalWheels[i] = m_verticalChecks[i].totalRejections();
  }
  counts.horizontalWheel = m_horizontalCheck.totalRejections();
  counts.leftDrive = m_leftDriveCheck.totalRejections();
  counts.rightDrive = m_rightDriveCheck.totalRejections();
  counts.imu = m_imuRejections;
  return counts;
}

}  // namespace cardLib
