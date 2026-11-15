// gps_sensor.cpp
// Why: implements GpsSensor on the PROS per-port GPS API (meters in, inches out).
// Owns: nothing beyond GpsSensor's own state.
#include "cardLib/hardware/gps_sensor.hpp"

#include <cmath>

#include "pros/gps.h"

namespace cardLib {

namespace {
constexpr double INCHES_PER_METER = 1000.0 / 25.4;
}  // namespace

std::optional<GpsSample> GpsSensor::read() {
  const pros::gps_position_s_t position = pros::c::gps_get_position(m_port);
  const double heading = pros::c::gps_get_heading(m_port);
  const double errorMeters = pros::c::gps_get_error(m_port);
  const bool ok = std::isfinite(position.x) && std::isfinite(position.y) &&
                  std::isfinite(heading) && std::isfinite(errorMeters);
  m_health.record(ok);
  if (!ok) {
    return std::nullopt;
  }
  return GpsSample{position.x * INCHES_PER_METER, position.y * INCHES_PER_METER, heading,
                   errorMeters * INCHES_PER_METER};
}

}  // namespace cardLib
