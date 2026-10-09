// rotation_sensor.hpp
// Why: thin wrapper over the V5 rotation sensor, the usual tracking-wheel encoder.
// Owns: the sensor's port, direction and health.
#pragma once

#include <cstdint>
#include <optional>

#include "cardLib/hardware/encoder.hpp"
#include "cardLib/hardware/sensor_health.hpp"

namespace cardLib {

class RotationSensor : public Encoder {
 public:
  RotationSensor(std::uint8_t port, bool reversed) : m_port(port), m_reversed(reversed) {}

  bool initialize() override;
  std::optional<double> readDegrees() override;  // centidegrees / 100
  bool reset() override;
  void registerPorts(PortRegistry& registry, const char* role,
                     Requirement requirement) const override;

  std::uint8_t port() const { return m_port; }
  const SensorHealth& health() const { return m_health; }

 private:
  std::uint8_t m_port;
  bool m_reversed;
  SensorHealth m_health;
};

}  // namespace cardLib
