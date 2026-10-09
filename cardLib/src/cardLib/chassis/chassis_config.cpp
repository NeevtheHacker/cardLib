// chassis_config.cpp
// Why: turns the config structs into the numbers the algorithms need.
// Owns: nothing; no state.
#include "cardLib/chassis/chassis_config.hpp"

#include "cardLib/algorithms/localization/wheel_distance.hpp"

namespace cardLib {

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
  odometry.effectiveTrackWidthInches =
      calibration.effectiveTrackWidthInches.value_or(config.geometry.trackWidthInches);
  odometry.velocitySmoothing = config.limits.velocitySmoothing;
  return odometry;
}

double maxDriveSpeedInchesPerSecond(const ChassisConfig& config) {
  double motorRpm = cartridgeRpm(Cartridge::Blue600);
  if (config.drive.left != nullptr && config.drive.left->size() > 0) {
    motorRpm = cartridgeRpm(config.drive.left->motor(0).cartridge);
  }
  return maxWheelSpeedInchesPerSecond(config.geometry.wheelDiameterInches,
                                      motorRpm * config.geometry.externalRatio);
}

}  // namespace cardLib
