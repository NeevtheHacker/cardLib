// distance_sensor.cpp
// Why: implements DistanceSensor on the PROS per-port distance API.
// Owns: nothing beyond DistanceSensor's own state.
#include "cardLib/hardware/distance_sensor.hpp"

#include "pros/distance.h"
#include "pros/error.h"

namespace cardLib {

namespace {
constexpr double MILLIMETERS_PER_INCH = 25.4;
constexpr std::int32_t NO_OBJECT_MILLIMETERS = 9999;  // what the sensor reports out of range
constexpr double MAX_CONFIDENCE = 63.0;
}  // namespace

std::optional<double> DistanceSensor::readInches() {
  const std::int32_t millimeters = pros::c::distance_get(m_port);
  const bool ok = millimeters != PROS_ERR;
  m_health.record(ok);
  if (!ok || millimeters >= NO_OBJECT_MILLIMETERS) {
    return std::nullopt;
  }
  return millimeters / MILLIMETERS_PER_INCH;
}

std::optional<double> DistanceSensor::readConfidence() {
  const std::int32_t confidence = pros::c::distance_get_confidence(m_port);
  if (confidence == PROS_ERR) {
    return std::nullopt;
  }
  return confidence / MAX_CONFIDENCE;
}

}  // namespace cardLib
