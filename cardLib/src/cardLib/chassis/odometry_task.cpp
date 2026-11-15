// odometry_task.cpp
// Why: the odometry loop: read every sensor once, build the SensorFrame, run the estimator,
//      publish (SPEC sections 2, 4 and 5).
// Owns: nothing beyond OdometryTask's own state.
#include "cardLib/chassis/odometry_task.hpp"

#include "cardLib/algorithms/localization/wheel_distance.hpp"
#include "cardLib/algorithms/math/loop_timing.hpp"
#include "cardLib/hardware/distance_sensor.hpp"
#include "cardLib/hardware/gps_sensor.hpp"
#include "cardLib/hardware/imu.hpp"

namespace cardLib {

namespace {
constexpr std::uint32_t POSE_TELEMETRY_EVERY_LOOPS = 10;  // 10 Hz
constexpr double MICROSECONDS_PER_SECOND = 1e6;
constexpr std::uint32_t RESET_POLL_MS = 2;
constexpr std::uint32_t IMU_FAILURES_TO_FAULT = 3;
constexpr std::uint32_t GPS_EVERY_LOOPS = 10;  // 10 Hz: resets only ever need a recent fix

std::array<PlausibilityCheck, MAX_WHEEL_MODULES> makeDriveChecks(double maxSpeed) {
  return {PlausibilityCheck(maxSpeed), PlausibilityCheck(maxSpeed), PlausibilityCheck(maxSpeed),
          PlausibilityCheck(maxSpeed), PlausibilityCheck(maxSpeed), PlausibilityCheck(maxSpeed),
          PlausibilityCheck(maxSpeed), PlausibilityCheck(maxSpeed)};
}
static_assert(MAX_WHEEL_MODULES == 8, "makeDriveChecks lists one check per module");
}  // namespace

OdometryTask::OdometryTask(const ChassisConfig& config, PoseEstimator& estimator,
                           TelemetryBuffer& telemetry)
    : m_config(config),
      m_drive(resolveDrive(config)),
      m_estimator(estimator),
      m_telemetry(telemetry),
      // The real speed limit is set in start(), once the motor groups exist. The chassis may
      // be a global built before them.
      m_verticalChecks{PlausibilityCheck(0.0), PlausibilityCheck(0.0)},
      m_horizontalCheck(0.0),
      m_driveChecks(makeDriveChecks(0.0)) {
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
  m_drive = resolveDrive(m_config);
  const double maxSpeed = maxDriveSpeedInchesPerSecond(m_config);
  m_verticalChecks = {PlausibilityCheck(maxSpeed), PlausibilityCheck(maxSpeed)};
  m_horizontalCheck = PlausibilityCheck(maxSpeed);
  m_driveChecks = makeDriveChecks(maxSpeed);
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
  for (std::size_t i = 0; i < m_drive.moduleCount; ++i) {
    m_driveChecks[i].rebaseline(moduleInches(i));
  }
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
  readGps();
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
  std::array<WheelDelta, MAX_WHEEL_MODULES> deltas = {};
  for (std::size_t i = 0; i < m_drive.moduleCount; ++i) {
    const auto inches = remember(moduleInches(i), m_totals.driveWheelInches[i]);
    deltas[i] = m_driveChecks[i].measure(inches, dtSeconds);
  }
  frame.driveWheels = deltas;
  frame.driveWheelCount = m_drive.moduleCount;
  combineSides(frame, deltas);
}

// A tank's modules become the left and right drive readings: the mean of each side's usable
// modules. A holonomic drive's corner wheels don't measure their side's travel, so it leaves
// both unusable.
void OdometryTask::combineSides(SensorFrame& frame,
                                const std::array<WheelDelta, MAX_WHEEL_MODULES>& deltas) {
  const bool tank = isTankDrive(m_drive);
  for (const Side side : {Side::Left, Side::Right}) {
    double deltaSum = 0.0;
    double totalSum = 0.0;
    std::size_t usable = 0;
    std::size_t onSide = 0;
    for (std::size_t i = 0; i < m_drive.moduleCount; ++i) {
      if (!moduleOnSide(i, side)) {
        continue;
      }
      totalSum += m_totals.driveWheelInches[i];
      ++onSide;
      if (deltas[i].usable) {
        deltaSum += deltas[i].inches;
        ++usable;
      }
    }
    WheelDelta& delta = side == Side::Left ? frame.leftDrive : frame.rightDrive;
    double& total = side == Side::Left ? m_totals.leftDriveInches : m_totals.rightDriveInches;
    if (onSide > 0) {
      total = totalSum / static_cast<double>(onSide);
    }
    if (tank && usable > 0) {
      delta = {deltaSum / static_cast<double>(usable), true};
    }
  }
}

bool OdometryTask::moduleOnSide(std::size_t module, Side side) const {
  const double x = m_drive.modules[module].xInches;
  return side == Side::Left ? x < 0.0 : x > 0.0;
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

// The GPS is read here so no other task touches a sensor this task owns. It feeds only the
// on-request reset (SPEC 21.3), so 10 Hz is plenty.
void OdometryTask::readGps() {
  const GpsConfig* config = m_config.sensors.gps;
  if (config == nullptr || config->sensor == nullptr || m_gpsLoop++ % GPS_EVERY_LOOPS != 0) {
    return;
  }
  const std::optional<GpsSample> sample = config->sensor->read();
  if (!sample.has_value()) {
    m_gps.valid = false;
    return;
  }
  const GpsMount mount{config->xInches,
                       config->yInches,
                       config->headingDegrees,
                       config->fieldXOffsetInches,
                       config->fieldYOffsetInches,
                       config->fieldRotationDegrees};
  m_gps = {.robotPose = robotPoseFromGps({sample->xInches, sample->yInches,
                                          sample->headingDegrees},
                                         mount),
           .errorInches = sample->errorInches,
           .valid = true};
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
      m_config.calibration.verticalWheelDiameterInches[index].value_or(wheel->diameterInches.value());
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
      m_config.calibration.horizontalWheelDiameterInches.value_or(wheel->diameterInches.value());
  return encoderDegreesToInches(*degrees, diameter, wheel->gearRatio);
}

std::optional<double> OdometryTask::moduleInches(std::size_t module) {
  const DriveModule& drive = m_drive.modules[module];
  if (drive.motors == nullptr) {
    return std::nullopt;
  }
  const std::optional<double> degrees = drive.motors->readDegrees();
  if (!degrees.has_value()) {
    return std::nullopt;
  }
  return motorEncoderInches(*degrees, drive.wheelDiameterInches, drive.gearRatio,
                            m_config.calibration.driveScale);
}

void OdometryTask::publish(const RobotState& state, const SensorFrame& frame) {
  bool leftFaulted = false;
  bool rightFaulted = false;
  bool motorMissing = false;
  for (std::size_t i = 0; i < m_drive.moduleCount; ++i) {
    const bool faulted = m_driveChecks[i].faulted();
    leftFaulted = leftFaulted || (faulted && moduleOnSide(i, Side::Left));
    rightFaulted = rightFaulted || (faulted && moduleOnSide(i, Side::Right));
    const MotorGroup* motors = m_drive.modules[i].motors;
    // A drive motor is missing only after three failed reads in a row, not on one glitch.
    motorMissing = motorMissing || (motors != nullptr && motors->anyMotorFaulted());
  }
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
    shared->leftDriveFaulted = leftFaulted;
    shared->rightDriveFaulted = rightFaulted;
    shared->driveMotorMissing = motorMissing;
    shared->distances = frame.distances;
    shared->gps = m_gps;
    const GpsConfig* gps = m_config.sensors.gps;
    shared->gpsFaulted =
        gps != nullptr && gps->sensor != nullptr && gps->sensor->health().faulted();
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
  for (std::size_t i = 0; i < m_drive.moduleCount; ++i) {
    const std::uint32_t rejections = m_driveChecks[i].totalRejections();
    counts.leftDrive += moduleOnSide(i, Side::Left) ? rejections : 0;
    counts.rightDrive += moduleOnSide(i, Side::Right) ? rejections : 0;
  }
  counts.imu = m_imuRejections;
  return counts;
}

}  // namespace cardLib
