// gps_sensor.hpp
// Why: thin wrapper over the V5 GPS sensor in inches and degrees, so a reset can snap the pose
//      to the field (SPEC 21.3).
// Owns: the sensor's port and health. Turning a sample into the robot's center in the team's
//       frame is algorithms/localization/pose_reset.
#pragma once

#include <cstdint>
#include <optional>

#include "cardLib/hardware/sensor_health.hpp"

namespace cardLib {

// What the sensor reports about itself: its position in the GPS field frame (origin at the
// field center, inches), its heading (clockwise, 0 to 360) and its error estimate (inches).
struct GpsSample {
  double xInches = 0.0;
  double yInches = 0.0;
  double headingDegrees = 0.0;
  double errorInches = 0.0;
};

class GpsSensor {
 public:
  explicit GpsSensor(std::uint8_t port) : m_port(port) {}

  // Empty if any read failed, including while the sensor is still calibrating.
  std::optional<GpsSample> read();

  std::uint8_t port() const { return m_port; }
  const SensorHealth& health() const { return m_health; }

 private:
  std::uint8_t m_port;
  SensorHealth m_health;
};

}  // namespace cardLib
