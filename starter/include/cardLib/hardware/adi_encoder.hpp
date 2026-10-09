// adi_encoder.hpp
// Why: thin wrapper over a 3-wire quadrature encoder (360 ticks per turn, so ticks = degrees).
// Owns: the encoder's ports, direction, PROS handle and health.
#pragma once

#include <cstdint>
#include <optional>

#include "cardLib/hardware/encoder.hpp"
#include "cardLib/hardware/sensor_health.hpp"

namespace cardLib {

class AdiEncoder : public Encoder {
 public:
  // Ports 1 to 8 ('A' to 'H' also work). The bottom port must be the top port + 1.
  AdiEncoder(std::uint8_t topPort, std::uint8_t bottomPort, bool reversed)
      : m_topPort(topPort), m_bottomPort(bottomPort), m_reversed(reversed) {}

  bool initialize() override;
  std::optional<double> readDegrees() override;
  bool reset() override;
  void registerPorts(PortRegistry& registry, const char* role,
                     Requirement requirement) const override;

  std::uint8_t topPort() const { return m_topPort; }
  std::uint8_t bottomPort() const { return m_bottomPort; }
  const SensorHealth& health() const { return m_health; }

 private:
  std::uint8_t m_topPort;
  std::uint8_t m_bottomPort;
  bool m_reversed;
  std::int32_t m_handle = 0;
  bool m_initialized = false;
  SensorHealth m_health;
};

}  // namespace cardLib
