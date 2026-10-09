// calibration_routines.cpp
// Why: implements the SPEC section 6 procedures and their printout.
// Owns: nothing beyond CalibrationRoutines' recorded runs.
#include "cardLib/systems/calibration_routines.hpp"

#include <cmath>
#include <cstdio>
#include <span>

#include "cardLib/algorithms/calibration/calibration_math.hpp"
#include "cardLib/algorithms/control/drive_mixer.hpp"
#include "cardLib/hardware/distance_sensor.hpp"

namespace cardLib {

namespace {

constexpr double SPIN_FRACTION = 0.25;  // slow, so the wheels don't slip (tune)
constexpr std::uint32_t POLL_MS = 10;
constexpr std::uint32_t STOP_SETTLE_MS = 500;

template <typename Run, typename Field>
RunValues collect(std::span<const Run> runs, Field field) {
  RunValues collected;
  for (const Run& run : runs) {
    const std::optional<double> value = field(run);
    if (value.has_value()) {
      collected.values[collected.count++] = *value;
    }
  }
  return collected;
}

// Median of the first three runs when there are three; the mean otherwise.
std::optional<double> combine(const RunValues& collected) {
  if (collected.count == 0) {
    return std::nullopt;
  }
  if (collected.count >= 3) {
    return medianOfThree(collected.values[0], collected.values[1], collected.values[2]);
  }
  double sum = 0.0;
  for (std::size_t i = 0; i < collected.count; ++i) {
    sum += collected.values[i];
  }
  return sum / static_cast<double>(collected.count);
}

bool scaleRunsAgree(const RunValues& collected) {
  return collected.count < 2 || scalesAgree(collected.values[0], collected.values[1]);
}

bool offsetRunsAgree(const RunValues& collected) {
  return collected.count < 2 || offsetsAgree(collected.values[0], collected.values[1]);
}

const TrackingWheelConfig* verticalWheel(const SensorConfig& sensors, std::size_t index) {
  if (sensors.verticalWheel1 == nullptr) {
    return index == 0 ? sensors.verticalWheel2 : nullptr;
  }
  return index == 0 ? sensors.verticalWheel1 : sensors.verticalWheel2;
}

}  // namespace

void CalibrationRoutines::beginRun() { m_start = m_chassis.odometrySnapshot().totals; }

bool CalibrationRoutines::spin(double turns, TurnDirection direction, std::uint32_t timeoutMs) {
  const double startDegrees = m_chassis.odometrySnapshot().totals.imuRotationDegrees;
  const double targetDegrees = 360.0 * std::abs(turns);
  const double sign = direction == TurnDirection::CounterClockwise ? -1.0 : 1.0;
  const auto millivolts = static_cast<std::int32_t>(sign * SPIN_FRACTION * MAX_MILLIVOLTS);
  m_chassis.driveMillivolts(millivolts, -millivolts);

  bool reached = false;
  for (std::uint32_t waitedMs = 0; waitedMs < timeoutMs && !reached; waitedMs += POLL_MS) {
    pros::delay(POLL_MS);
    const double turned =
        m_chassis.odometrySnapshot().totals.imuRotationDegrees - startDegrees;
    reached = std::abs(turned) >= targetDegrees;
  }
  m_chassis.driveMillivolts(0, 0);
  pros::delay(STOP_SETTLE_MS);
  return reached;
}

SensorTotals CalibrationRoutines::changeSinceStart() const {
  const SensorTotals now = m_chassis.odometrySnapshot().totals;
  SensorTotals change;
  for (std::size_t i = 0; i < MAX_VERTICAL_WHEELS; ++i) {
    change.verticalWheelInches[i] = now.verticalWheelInches[i] - m_start.verticalWheelInches[i];
  }
  change.horizontalWheelInches = now.horizontalWheelInches - m_start.horizontalWheelInches;
  change.leftDriveInches = now.leftDriveInches - m_start.leftDriveInches;
  change.rightDriveInches = now.rightDriveInches - m_start.rightDriveInches;
  change.imuRotationDegrees = now.imuRotationDegrees - m_start.imuRotationDegrees;
  return change;
}

void CalibrationRoutines::recordImuRun(double turns) {
  const double measured = changeSinceStart().imuRotationDegrees;
  m_imu.turns = std::abs(turns);
  if (measured > 0.0) {
    m_imu.clockwiseDegrees = measured;
  } else {
    m_imu.counterclockwiseDegrees = measured;
  }
}

double CalibrationRoutines::currentVerticalDiameter(std::size_t index) const {
  const ChassisConfig& config = m_chassis.config();
  const TrackingWheelConfig* wheel = verticalWheel(config.sensors, index);
  const double nominal = wheel != nullptr ? wheel->diameterInches : 0.0;
  return config.calibration.verticalWheelDiameterInches[index].value_or(nominal);
}

double CalibrationRoutines::currentHorizontalDiameter() const {
  const ChassisConfig& config = m_chassis.config();
  const TrackingWheelConfig* wheel = config.sensors.horizontalWheel;
  const double nominal = wheel != nullptr ? wheel->diameterInches : 0.0;
  return config.calibration.horizontalWheelDiameterInches.value_or(nominal);
}

void CalibrationRoutines::recordPushRun(double tapedInches) {
  const SensorTotals change = changeSinceStart();
  PushRun run;
  for (std::size_t i = 0; i < MAX_VERTICAL_WHEELS; ++i) {
    if (verticalWheel(m_chassis.config().sensors, i) != nullptr) {
      run.verticalDiameterInches[i] = trackingWheelDiameter(
          currentVerticalDiameter(i), tapedInches, std::abs(change.verticalWheelInches[i]));
    }
  }
  const double driveInches =
      (std::abs(change.leftDriveInches) + std::abs(change.rightDriveInches)) / 2.0;
  if (const std::optional<double> scale = driveScale(tapedInches, driveInches)) {
    run.driveScale = *scale * m_chassis.config().calibration.driveScale;
  }
  m_pushes.add(run);
}

void CalibrationRoutines::recordSidewaysPushRun(double tapedInches) {
  const double measured = std::abs(changeSinceStart().horizontalWheelInches);
  if (const auto diameter =
          trackingWheelDiameter(currentHorizontalDiameter(), tapedInches, measured)) {
    m_sidewaysDiameters.add(*diameter);
  }
}

void CalibrationRoutines::recordSpinRun() {
  const SensorTotals change = changeSinceStart();
  const ChassisConfig& config = m_chassis.config();
  const double headingRadians =
      degreesToRadians(config.calibration.imuScale * change.imuRotationDegrees);
  SpinRun run;
  for (std::size_t i = 0; i < MAX_VERTICAL_WHEELS; ++i) {
    if (verticalWheel(config.sensors, i) != nullptr) {
      run.verticalXInches[i] =
          verticalWheelOffset(change.verticalWheelInches[i], headingRadians);
    }
  }
  if (config.sensors.horizontalWheel != nullptr) {
    run.horizontalYInches = horizontalWheelOffset(change.horizontalWheelInches, headingRadians);
  }
  run.effectiveTrackWidthInches =
      effectiveTrackWidth(change.leftDriveInches, change.rightDriveInches, headingRadians);
  m_spins.add(run);
}

bool CalibrationRoutines::addDistancePoint(std::size_t sensorIndex, double trueInches) {
  if (sensorIndex >= MAX_DISTANCE_SENSORS) {
    return false;
  }
  const DistanceSensorConfig* config = m_chassis.config().sensors.distanceSensors[sensorIndex];
  const DistanceReading reading = m_chassis.odometrySnapshot().distances[sensorIndex];
  DistancePoints& points = m_distancePoints[sensorIndex];
  if (config == nullptr || !reading.valid || config->gain == 0.0 ||
      points.count == MAX_DISTANCE_POINTS) {
    return false;
  }
  points.raw[points.count] = (reading.inches - config->offsetInches) / config->gain;
  points.truth[points.count] = trueInches;
  ++points.count;
  return true;
}

std::optional<LineFit> CalibrationRoutines::distanceFit(std::size_t sensorIndex) const {
  if (sensorIndex >= MAX_DISTANCE_SENSORS) {
    return std::nullopt;
  }
  const DistancePoints& points = m_distancePoints[sensorIndex];
  return fitLine(std::span<const double>(points.raw.data(), points.count),
                 std::span<const double>(points.truth.data(), points.count));
}

// Every value measured so far, combined across runs.
CalibrationRoutines::Measured CalibrationRoutines::measure() const {
  const std::span<const PushRun> pushes(m_pushes.values.data(), m_pushes.count);
  const std::span<const SpinRun> spins(m_spins.values.data(), m_spins.count);
  Measured measured;
  for (std::size_t i = 0; i < MAX_VERTICAL_WHEELS; ++i) {
    const auto diameterOf = [i](const PushRun& run) { return run.verticalDiameterInches[i]; };
    const auto offsetOf = [i](const SpinRun& run) { return run.verticalXInches[i]; };
    measured.verticalDiameters[i] = collect(pushes, diameterOf);
    measured.verticalOffsets[i] = collect(spins, offsetOf);
  }
  measured.driveScales = collect(pushes, [](const PushRun& run) { return run.driveScale; });
  measured.horizontalDiameters.values = m_sidewaysDiameters.values;
  measured.horizontalDiameters.count = m_sidewaysDiameters.count;
  measured.horizontalOffsets =
      collect(spins, [](const SpinRun& run) { return run.horizontalYInches; });
  measured.trackWidths =
      collect(spins, [](const SpinRun& run) { return run.effectiveTrackWidthInches; });
  return measured;
}

Calibration CalibrationRoutines::results() const {
  Calibration calibration = m_chassis.config().calibration;
  calibration.source = "measured";
  if (m_imu.clockwiseDegrees && m_imu.counterclockwiseDegrees) {
    calibration.imuScale =
        imuScale(m_imu.turns, *m_imu.clockwiseDegrees, *m_imu.counterclockwiseDegrees)
            .value_or(calibration.imuScale);
  }
  const Measured measured = measure();
  for (std::size_t i = 0; i < MAX_VERTICAL_WHEELS; ++i) {
    if (const auto diameter = combine(measured.verticalDiameters[i])) {
      calibration.verticalWheelDiameterInches[i] = diameter;
    }
    if (const auto offset = combine(measured.verticalOffsets[i])) {
      calibration.verticalWheelXInches[i] = offset;
    }
  }
  calibration.driveScale = combine(measured.driveScales).value_or(calibration.driveScale);
  if (const auto diameter = combine(measured.horizontalDiameters)) {
    calibration.horizontalWheelDiameterInches = diameter;
  }
  if (const auto offset = combine(measured.horizontalOffsets)) {
    calibration.horizontalWheelYInches = offset;
  }
  if (const auto width = combine(measured.trackWidths)) {
    calibration.effectiveTrackWidthInches = width;
  }
  return calibration;
}

void CalibrationRoutines::printValue(const char* name, std::optional<double> value,
                                     bool runsAgree) const {
  if (!value.has_value()) {
    return;
  }
  std::printf("calibration.%s = %.4f;  // %s\n", name, *value,
              runsAgree ? "runs agree" : "runs DISAGREE: check the mechanics");
}

void CalibrationRoutines::printResults() const {
  const Calibration calibration = results();
  const Measured measured = measure();
  std::printf("// ---- cardLib calibration: paste into calibration_values.hpp ----\n");
  if (m_imu.clockwiseDegrees && m_imu.counterclockwiseDegrees) {
    const double clockwise = 360.0 * m_imu.turns / std::abs(*m_imu.clockwiseDegrees);
    const double counter = 360.0 * m_imu.turns / std::abs(*m_imu.counterclockwiseDegrees);
    printValue("imuScale", calibration.imuScale, scalesAgree(clockwise, counter));
  }
  if (measured.driveScales.count > 0) {
    printValue("driveScale", calibration.driveScale, scaleRunsAgree(measured.driveScales));
  }
  const char* const diameterNames[] = {"verticalWheelDiameterInches[0]",
                                       "verticalWheelDiameterInches[1]"};
  const char* const offsetNames[] = {"verticalWheelXInches[0]", "verticalWheelXInches[1]"};
  for (std::size_t i = 0; i < MAX_VERTICAL_WHEELS; ++i) {
    printValue(diameterNames[i], combine(measured.verticalDiameters[i]),
               scaleRunsAgree(measured.verticalDiameters[i]));
    printValue(offsetNames[i], combine(measured.verticalOffsets[i]),
               offsetRunsAgree(measured.verticalOffsets[i]));
  }
  printValue("horizontalWheelDiameterInches", combine(measured.horizontalDiameters),
             scaleRunsAgree(measured.horizontalDiameters));
  printValue("horizontalWheelYInches", combine(measured.horizontalOffsets),
             offsetRunsAgree(measured.horizontalOffsets));
  printValue("effectiveTrackWidthInches", combine(measured.trackWidths),
             offsetRunsAgree(measured.trackWidths));
  for (std::size_t i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
    if (const std::optional<LineFit> fit = distanceFit(i)) {
      std::printf("// distance sensor %u: .gain = %.4f, .offsetInches = %.3f\n",
                  static_cast<unsigned>(i), fit->gain, fit->offset);
    }
  }
}

void CalibrationRoutines::clear() {
  m_start = {};
  m_imu = {};
  m_pushes = {};
  m_sidewaysDiameters = {};
  m_spins = {};
  m_distancePoints = {};
}

}  // namespace cardLib
