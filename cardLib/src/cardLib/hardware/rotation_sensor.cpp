// rotation_sensor.cpp
// Why: implements RotationSensor on the PROS per-port rotation API.
// Owns: nothing beyond RotationSensor's own state.
#include "cardLib/hardware/rotation_sensor.hpp"

#include "pros/error.h"
#include "pros/rotation.h"

namespace cardLib {

namespace {
constexpr double CENTIDEGREES_PER_DEGREE = 100.0;
}

bool RotationSensor::initialize() {
  const bool ok = pros::c::rotation_set_reversed(m_port, m_reversed) != PROS_ERR;
  m_health.record(ok);
  return ok;
}

std::optional<double> RotationSensor::readDegrees() {
  const std::int32_t centidegrees = pros::c::rotation_get_position(m_port);
  const bool ok = centidegrees != PROS_ERR;
  m_health.record(ok);
  if (!ok) {
    return std::nullopt;
  }
  return centidegrees / CENTIDEGREES_PER_DEGREE;
}

bool RotationSensor::reset() { return pros::c::rotation_reset_position(m_port) != PROS_ERR; }

void RotationSensor::registerPorts(PortRegistry& registry, const char* role,
                                   Requirement requirement) const {
  registry.add(m_port, DeviceKind::Rotation, role, requirement);
}

}  // namespace cardLib
