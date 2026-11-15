// sensor_status.cpp
// Why: implements the SPEC 21.4 sensor statuses from the startup report and odometry snapshot.
// Owns: nothing; no state.
#include "cardLib/chassis/sensor_status.hpp"

#include <cstring>

#include "cardLib/hardware/distance_sensor.hpp"
#include "cardLib/hardware/gps_sensor.hpp"

namespace cardLib {

namespace {

// Short names, two to a line on the brain screen.
constexpr const char* DISTANCE_NAMES[MAX_DISTANCE_SENSORS] = {"dist 1", "dist 2", "dist 3",
                                                              "dist 4"};
// A GPS fix worse than this is shown as Suspect (the reset's default limit).
constexpr double GPS_SUSPECT_ERROR_INCHES = 1.0;

// What startup found for this role's device, or for this port. Missing wins over Wrong.
enum class StartupProblem { None, Wrong, Missing };

StartupProblem startupProblem(const StartupReport& startup, const char* role, int port = -1) {
  StartupProblem worst = StartupProblem::None;
  for (std::size_t i = 0; i < startup.portIssueCount; ++i) {
    const PortIssue& issue = startup.portIssues[i];
    const bool roleMatches = role != nullptr && std::strcmp(issue.role, role) == 0;
    if (!roleMatches && static_cast<int>(issue.port) != port) {
      continue;
    }
    if (issue.problem == PortProblem::Missing) {
      return StartupProblem::Missing;
    }
    worst = StartupProblem::Wrong;
  }
  return worst;
}

StartupProblem worse(StartupProblem a, StartupProblem b) {
  return static_cast<int>(a) > static_cast<int>(b) ? a : b;
}

SensorStatus statusOf(StartupProblem problem, bool faulted, bool suspect) {
  if (problem == StartupProblem::Missing) {
    return SensorStatus::Missing;
  }
  if (problem == StartupProblem::Wrong) {
    return SensorStatus::Wrong;
  }
  if (faulted) {
    return SensorStatus::Faulted;
  }
  return suspect ? SensorStatus::Suspect : SensorStatus::Ok;
}

void add(SensorStatusList& list, const char* name, SensorStatus status) {
  if (list.count < list.lines.size()) {
    list.lines[list.count++] = {name, status};
  }
}

void addTrackingWheels(SensorStatusList& list, const SensorConfig& sensors,
                       const StartupReport& startup, const OdometrySnapshot& odometry) {
  const bool slip = odometry.state.wheelHeadingSuspect;
  const std::array<const TrackingWheelConfig*, MAX_VERTICAL_WHEELS> verticals{
      sensors.verticalWheel1, sensors.verticalWheel2};
  constexpr const char* ROLES[MAX_VERTICAL_WHEELS] = {"vertical wheel 1", "vertical wheel 2"};
  constexpr const char* NAMES[MAX_VERTICAL_WHEELS] = {"vert 1", "vert 2"};
  std::size_t index = 0;
  for (std::size_t i = 0; i < MAX_VERTICAL_WHEELS; ++i) {
    if (verticals[i] == nullptr) {
      continue;
    }
    add(list, NAMES[i],
        statusOf(startupProblem(startup, ROLES[i]), odometry.verticalWheelFaulted[index], slip));
    ++index;
  }
  if (sensors.horizontalWheel != nullptr) {
    add(list, "horiz",
        statusOf(startupProblem(startup, "horizontal wheel"), odometry.horizontalWheelFaulted,
                 false));
  }
}

void addDrive(SensorStatusList& list, const ChassisConfig& config, const StartupReport& startup,
              const OdometrySnapshot& odometry) {
  const DriveConfig drive = resolveDrive(config);
  if (drive.moduleCount == 0) {
    return;
  }
  StartupProblem problem =
      startup.modulesWithoutMotors > 0 ? StartupProblem::Missing : StartupProblem::None;
  for (std::size_t i = 0; i < drive.moduleCount; ++i) {
    const MotorGroup* motors = drive.modules[i].motors;
    for (std::size_t m = 0; motors != nullptr && m < motors->size(); ++m) {
      problem = worse(problem, startupProblem(startup, nullptr, motors->motor(m).port));
    }
  }
  const bool encoderFault = odometry.leftDriveFaulted || odometry.rightDriveFaulted ||
                            odometry.state.driveSlipSuspect;
  add(list, "drive", statusOf(problem, odometry.driveMotorMissing, encoderFault));
}

void addFieldSensors(SensorStatusList& list, const SensorConfig& sensors,
                     const StartupReport& startup, const OdometrySnapshot& odometry) {
  for (std::size_t i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
    const DistanceSensorConfig* distance = sensors.distanceSensors[i];
    if (distance == nullptr || distance->sensor == nullptr) {
      continue;
    }
    add(list, DISTANCE_NAMES[i],
        statusOf(startupProblem(startup, nullptr, distance->sensor->port()), false,
                 !odometry.distances[i].valid));
  }
  if (sensors.gps != nullptr && sensors.gps->sensor != nullptr) {
    const bool poorFix = !odometry.gps.valid || odometry.gps.errorInches > GPS_SUSPECT_ERROR_INCHES;
    add(list, "GPS",
        statusOf(startupProblem(startup, nullptr, sensors.gps->sensor->port()),
                 odometry.gpsFaulted, poorFix));
  }
}

}  // namespace

SensorStatusList sensorStatus(const ChassisConfig& config, const StartupReport& startup,
                              const OdometrySnapshot& odometry) {
  SensorStatusList list;
  const SensorConfig& sensors = config.sensors;
  if (sensors.imu != nullptr && sensors.imu->sensor != nullptr) {
    const bool imuDoubtful = !odometry.imuAnswering || startup.imuCalibrationFailed ||
                             startup.imuDriftTooHigh || startup.imuOrientationMismatch;
    add(list, "IMU", statusOf(startupProblem(startup, "IMU"), odometry.imuFaulted, imuDoubtful));
  }
  addTrackingWheels(list, sensors, startup, odometry);
  addDrive(list, config, startup, odometry);
  addFieldSensors(list, sensors, startup, odometry);
  return list;
}

const char* sensorStatusName(SensorStatus status) {
  switch (status) {
    case SensorStatus::Ok:
      return "OK";
    case SensorStatus::Suspect:
      return "Suspect";
    case SensorStatus::Faulted:
      return "Faulted";
    case SensorStatus::Wrong:
      return "Wrong";
    case SensorStatus::Missing:
      return "Missing";
  }
  return "?";
}

}  // namespace cardLib
