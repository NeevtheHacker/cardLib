// distance_sensor.hpp
// Why: thin wrapper over the V5 distance sensor, in inches, with its confidence.
// Owns: the sensor's port and health. Gain/offset correction lives with its config.
#pragma once

#include <cstdint>
#include <optional>

#include "cardLib/hardware/sensor_health.hpp"

namespace cardLib {

class DistanceSensor {
 public:
  explicit DistanceSensor(std::uint8_t port) : m_port(port) {}

  // Raw distance, empty if the read failed or nothing is in range.
  std::optional<double> readInches();

  // 0 to 1. Empty if the read failed.
  std::optional<double> readConfidence();

  std::uint8_t port() const { return m_port; }
  const SensorHealth& health() const { return m_health; }

 private:
  std::uint8_t m_port;
  SensorHealth m_health;
};

}  // namespace cardLib
