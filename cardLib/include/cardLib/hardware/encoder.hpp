// encoder.hpp
// Why: one interface for anything that measures rotation: motors, rotation sensors, ADI encoders.
// Owns: the Encoder interface (SPEC section 17).
#pragma once

#include <optional>

#include "cardLib/hardware/hardware_config.hpp"
#include "cardLib/hardware/port_registry.hpp"

namespace cardLib {

class Encoder {
 public:
  virtual ~Encoder() = default;

  // Configures the device. Call once from initialize(). False if the device didn't answer.
  virtual bool initialize() = 0;

  // Cumulative shaft angle in degrees, or empty if the read failed. Read once per loop.
  virtual std::optional<double> readDegrees() = 0;

  // Zeroes the angle. False if the device didn't answer.
  virtual bool reset() = 0;

  // Adds this device's ports to the startup check.
  virtual void registerPorts(PortRegistry& registry, const char* role,
                             Requirement requirement) const = 0;
};

}  // namespace cardLib
