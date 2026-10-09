// sensor_frame.hpp
// Why: every reading taken this loop in one struct, so estimators never touch hardware (SPEC 18).
// Owns: SensorFrame. Filled by buildSensorFrame() in the odometry task.
#pragma once

#include <array>
#include <cstddef>

#include "cardLib/algorithms/math/loop_timing.hpp"

namespace cardLib {

constexpr std::size_t MAX_VERTICAL_WHEELS = 2;
constexpr std::size_t MAX_DISTANCE_SENSORS = 4;

// Distance a wheel rolled since last loop. valid = false: the reading failed or was rejected
// this loop, and inches is 0 (never a reused old delta).
struct WheelDelta {
  double inches = 0.0;
  bool valid = false;
};

// A distance sensor reading, already corrected by its calibrated gain and offset.
struct DistanceReading {
  double inches = 0.0;
  double confidence = 0.0;  // 0 to 1
  bool valid = false;
};

struct SensorFrame {
  double dtSeconds = LOOP_PERIOD_SECONDS;

  // Tracking wheels, in the order they were configured. Forward / right positive.
  std::array<WheelDelta, MAX_VERTICAL_WHEELS> verticalWheels = {};
  WheelDelta horizontalWheel = {};

  // Drive motor encoders, already scaled by the external ratio and drive scale.
  WheelDelta leftDrive = {};
  WheelDelta rightDrive = {};

  // Raw IMU rotation (unscaled, clockwise positive, continuous) and turn rate.
  bool imuValid = false;
  double imuRotationDegrees = 0.0;
  double imuGyroRateDegreesPerSecond = 0.0;

  std::array<DistanceReading, MAX_DISTANCE_SENSORS> distances = {};
};

}  // namespace cardLib
