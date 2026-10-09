// adi_encoder.cpp
// Why: implements AdiEncoder on the PROS ADI encoder API.
// Owns: nothing beyond AdiEncoder's own state.
#include "cardLib/hardware/adi_encoder.hpp"

#include "pros/adi.h"
#include "pros/error.h"

namespace cardLib {

bool AdiEncoder::initialize() {
  m_handle = pros::c::adi_encoder_init(m_topPort, m_bottomPort, m_reversed);
  m_initialized = m_handle != PROS_ERR;
  m_health.record(m_initialized);
  return m_initialized;
}

std::optional<double> AdiEncoder::readDegrees() {
  if (!m_initialized) {
    m_health.record(false);
    return std::nullopt;
  }
  const std::int32_t ticks = pros::c::adi_encoder_get(m_handle);
  const bool ok = ticks != PROS_ERR;
  m_health.record(ok);
  if (!ok) {
    return std::nullopt;
  }
  return static_cast<double>(ticks);
}

bool AdiEncoder::reset() {
  return m_initialized && pros::c::adi_encoder_reset(m_handle) != PROS_ERR;
}

void AdiEncoder::registerPorts(PortRegistry& registry, const char* role,
                               Requirement requirement) const {
  registry.add(m_topPort, DeviceKind::AdiEncoder, role, requirement);
  registry.add(m_bottomPort, DeviceKind::AdiEncoderBottom, role, requirement);
}

}  // namespace cardLib
