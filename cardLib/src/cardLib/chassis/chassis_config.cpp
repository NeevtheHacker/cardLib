// chassis_config.cpp
// Why: turns the config structs into the numbers the algorithms need.
// Owns: nothing; no state.
#include "cardLib/chassis/chassis_config.hpp"

#include <algorithm>

#include "cardLib/algorithms/localization/wheel_distance.hpp"

namespace cardLib {

namespace {

// Mean x of the modules on one side of center, and how many there are.
struct SideMean {
  double xInches = 0.0;
  std::size_t count = 0;
};

SideMean sideMean(const DriveConfig& drive, bool leftSide) {
  SideMean mean;
  for (std::size_t i = 0; i < drive.moduleCount; ++i) {
    const double x = drive.modules[i].xInches;
    if ((leftSide && x < 0.0) || (!leftSide && x > 0.0)) {
      mean.xInches += x;
      ++mean.count;
    }
  }
  if (mean.count > 0) {
    mean.xInches /= static_cast<double>(mean.count);
  }
  return mean;
}

}  // namespace

DriveConfig resolveDrive(const ChassisConfig& config) {
  if (config.drive.moduleCount > 0) {
    return config.drive;
  }
  DriveConfig drive = config.drive;
  const Geometry& geometry = config.geometry;
  const double halfTrack = geometry.trackWidthInches / 2.0;
  if (drive.left != nullptr) {
    drive.modules[drive.moduleCount++] = {
        drive.left, geometry.wheelDiameterInches, geometry.externalRatio, -halfTrack, 0.0, 0.0};
  }
  if (drive.right != nullptr) {
    drive.modules[drive.moduleCount++] = {
        drive.right, geometry.wheelDiameterInches, geometry.externalRatio, halfTrack, 0.0, 0.0};
  }
  return drive;
}

bool isTankDrive(const DriveConfig& drive) {
  for (std::size_t i = 0; i < drive.moduleCount; ++i) {
    if (drive.modules[i].rollAngleDegrees != 0.0) {
      return false;
    }
  }
  return true;
}

double driveTrackWidthInches(const DriveConfig& drive) {
  const SideMean left = sideMean(drive, true);
  const SideMean right = sideMean(drive, false);
  if (left.count == 0 || right.count == 0) {
    return 0.0;
  }
  return right.xInches - left.xInches;
}

double moduleMotorRpm(const DriveModule& module) {
  if (module.motors == nullptr || module.motors->size() == 0) {
    return cartridgeRpm(Cartridge::Blue600);
  }
  return motorRpm(module.motors->motor(0));
}

double moduleTopSpeedInchesPerSecond(const DriveModule& module) {
  return maxWheelSpeedInchesPerSecond(module.wheelDiameterInches,
                                      moduleMotorRpm(module) * module.gearRatio);
}

// The drive's rows from positions and roll angles alone (top speeds of 1), for odometry's
// forward kinematics, which never needs speeds. Reads no motor, so it is safe in constructors.
DriveKinematics driveGeometryKinematics(const DriveConfig& drive) {
  std::array<WheelSpec, MAX_WHEEL_MODULES> wheels = {};
  const std::size_t count = std::min(drive.moduleCount, MAX_WHEEL_MODULES);
  for (std::size_t i = 0; i < count; ++i) {
    const DriveModule& module = drive.modules[i];
    wheels[i] = {module.xInches, module.yInches, module.rollAngleDegrees, 1.0};
  }
  return DriveKinematics(std::span<const WheelSpec>(wheels.data(), count));
}

DriveKinematics makeDriveKinematics(const DriveConfig& drive) {
  std::array<WheelSpec, MAX_WHEEL_MODULES> wheels = {};
  const std::size_t count = std::min(drive.moduleCount, MAX_WHEEL_MODULES);
  for (std::size_t i = 0; i < count; ++i) {
    const DriveModule& module = drive.modules[i];
    wheels[i] = {module.xInches, module.yInches, module.rollAngleDegrees,
                 moduleTopSpeedInchesPerSecond(module)};
  }
  return DriveKinematics(std::span<const WheelSpec>(wheels.data(), count));
}

ArcOdometryConfig makeArcOdometryConfig(const ChassisConfig& config) {
  const SensorConfig& sensors = config.sensors;
  const Calibration& calibration = config.calibration;
  ArcOdometryConfig odometry;

  const std::array<const TrackingWheelConfig*, MAX_VERTICAL_WHEELS> verticals{
      sensors.verticalWheel1, sensors.verticalWheel2};
  for (const TrackingWheelConfig* wheel : verticals) {
    if (wheel == nullptr) {
      continue;
    }
    const std::size_t index = odometry.verticalWheelCount++;
    odometry.verticalWheelXInches[index] =
        calibration.verticalWheelXInches[index].value_or(wheel->xInches);
  }
  if (sensors.horizontalWheel != nullptr) {
    odometry.hasHorizontalWheel = true;
    odometry.horizontalWheelYInches =
        calibration.horizontalWheelYInches.value_or(sensors.horizontalWheel->yInches);
  }
  odometry.hasImu = sensors.imu != nullptr;
  odometry.imuScale = calibration.imuScale;
  const double driveTrackWidth = driveTrackWidthInches(resolveDrive(config));
  odometry.effectiveTrackWidthInches = calibration.effectiveTrackWidthInches.value_or(
      driveTrackWidth > 0.0 ? driveTrackWidth : config.geometry.trackWidthInches);
  odometry.velocitySmoothing = config.limits.velocitySmoothing;
  odometry.driveKinematics = driveGeometryKinematics(resolveDrive(config));
  return odometry;
}

double maxDriveSpeedInchesPerSecond(const ChassisConfig& config) {
  const DriveConfig drive = resolveDrive(config);
  double fastest = 0.0;
  for (std::size_t i = 0; i < drive.moduleCount; ++i) {
    fastest = std::max(fastest, moduleTopSpeedInchesPerSecond(drive.modules[i]));
  }
  if (fastest > 0.0) {
    return fastest;
  }
  return maxWheelSpeedInchesPerSecond(
      config.geometry.wheelDiameterInches,
      cartridgeRpm(Cartridge::Blue600) * config.geometry.externalRatio);
}

}  // namespace cardLib
