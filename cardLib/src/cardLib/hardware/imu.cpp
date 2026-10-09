// imu.cpp
// Why: implements Imu on the PROS per-port IMU API.
// Owns: nothing beyond Imu's own state.
#include "cardLib/hardware/imu.hpp"

#include <cmath>

#include "pros/error.h"
#include "pros/imu.h"

namespace cardLib {

bool Imu::startCalibration() {
  const bool ok = pros::c::imu_reset(m_port) != PROS_ERR;
  m_health.record(ok);
  return ok;
}

bool Imu::isCalibrating() const {
  const pros::imu_status_e_t status = pros::c::imu_get_status(m_port);
  return status != pros::E_IMU_STATUS_ERROR && (status & pros::E_IMU_STATUS_CALIBRATING) != 0;
}

std::optional<double> Imu::readRotationDegrees() {
  const double rotation = pros::c::imu_get_rotation(m_port);
  const bool ok = std::isfinite(rotation);
  m_health.record(ok);
  if (!ok) {
    return std::nullopt;
  }
  return rotation;
}

std::optional<double> Imu::readGyroRateDegreesPerSecond() {
  const double rate = pros::c::imu_get_gyro_rate(m_port).z;
  if (!std::isfinite(rate)) {
    return std::nullopt;
  }
  return rate;
}

std::optional<ImuOrientation> Imu::physicalOrientation() const {
  switch (pros::c::imu_get_physical_orientation(m_port)) {
    case pros::E_IMU_Z_DOWN:
      return ImuOrientation::Flat;
    case pros::E_IMU_Z_UP:
      return ImuOrientation::UpsideDown;
    case pros::E_IMU_X_UP:
      return ImuOrientation::XUp;
    case pros::E_IMU_X_DOWN:
      return ImuOrientation::XDown;
    case pros::E_IMU_Y_UP:
      return ImuOrientation::YUp;
    case pros::E_IMU_Y_DOWN:
      return ImuOrientation::YDown;
    default:
      return std::nullopt;
  }
}

}  // namespace cardLib
