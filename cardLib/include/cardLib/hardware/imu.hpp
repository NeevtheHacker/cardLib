// imu.hpp
// Why: thin wrapper over the V5 inertial sensor; every read returns empty on failure (SPEC 4).
// Owns: the IMU's port and health. Scaling (k_imu) is applied in algorithms/, not here.
#pragma once

#include <cstdint>
#include <optional>

#include "cardLib/hardware/hardware_config.hpp"
#include "cardLib/hardware/sensor_health.hpp"

namespace cardLib {

class Imu {
 public:
  explicit Imu(std::uint8_t port) : m_port(port) {}

  // Starts calibration without blocking. False if the IMU didn't answer.
  bool startCalibration();
  bool isCalibrating() const;

  // Raw rotation: continuous, clockwise positive, unscaled.
  std::optional<double> readRotationDegrees();

  // Raw z-axis turn rate as the IMU reports it. Its sign must pass the sign check before the
  // gyro-rate D term is used (SPEC section 7).
  std::optional<double> readGyroRateDegreesPerSecond();

  // How the IMU detects it is mounted; compared with the configured orientation at startup.
  std::optional<ImuOrientation> physicalOrientation() const;

  std::uint8_t port() const { return m_port; }
  const SensorHealth& health() const { return m_health; }

 private:
  std::uint8_t m_port;
  SensorHealth m_health;
};

}  // namespace cardLib
