// motor_group.cpp
// Why: implements MotorGroup on the PROS per-port motor API (no vectors, no allocation).
// Owns: nothing beyond MotorGroup's own state.
#include "cardLib/hardware/motor_group.hpp"

#include <cmath>

#include "pros/error.h"
#include "pros/motors.h"

namespace cardLib {

namespace {

pros::motor_gearset_e_t toGearset(Cartridge cartridge) {
  switch (cartridge) {
    case Cartridge::Red100:
      return pros::E_MOTOR_GEAR_RED;
    case Cartridge::Green200:
      return pros::E_MOTOR_GEAR_GREEN;
    case Cartridge::Blue600:
      return pros::E_MOTOR_GEAR_BLUE;
  }
  return pros::E_MOTOR_GEAR_BLUE;
}

pros::motor_brake_mode_e_t toBrakeMode(BrakeMode mode) {
  switch (mode) {
    case BrakeMode::Coast:
      return pros::E_MOTOR_BRAKE_COAST;
    case BrakeMode::Brake:
      return pros::E_MOTOR_BRAKE_BRAKE;
    case BrakeMode::Hold:
      return pros::E_MOTOR_BRAKE_HOLD;
  }
  return pros::E_MOTOR_BRAKE_COAST;
}

}  // namespace

MotorGroup::MotorGroup(std::initializer_list<MotorConfig> motors, const char* role)
    : m_role(role) {
  for (const MotorConfig& motor : motors) {
    if (m_count == MAX_MOTORS_PER_GROUP) {
      m_tooManyMotors = true;
      break;
    }
    m_motors[m_count++] = motor;
  }
}

std::int8_t MotorGroup::signedPort(std::size_t index) const {
  const auto port = static_cast<std::int8_t>(m_motors[index].port);
  return m_motors[index].reversed ? static_cast<std::int8_t>(-port) : port;
}

bool MotorGroup::initialize() {
  bool allOk = !m_tooManyMotors && m_count > 0;
  for (std::size_t i = 0; i < m_count; ++i) {
    const std::int8_t port = signedPort(i);
    const bool unitsOk =
        pros::c::motor_set_encoder_units(port, pros::E_MOTOR_ENCODER_DEGREES) != PROS_ERR;
    const bool gearOk =
        pros::c::motor_set_gearing(port, toGearset(effectiveCartridge(m_motors[i]))) != PROS_ERR;
    m_health[i].record(unitsOk && gearOk);
    allOk = allOk && unitsOk && gearOk;
  }
  return allOk;
}

std::optional<double> MotorGroup::readDegrees() {
  double changeSum = 0.0;
  std::size_t changeCount = 0;
  m_answeringCount = 0;
  for (std::size_t i = 0; i < m_count; ++i) {
    const double degrees = pros::c::motor_get_position(signedPort(i));
    const bool ok = std::isfinite(degrees);
    m_health[i].record(ok);
    if (!ok) {
      m_hasLast[i] = false;
      continue;
    }
    ++m_answeringCount;
    if (m_hasLast[i]) {
      changeSum += degrees - m_lastDegrees[i];
      ++changeCount;
    }
    m_lastDegrees[i] = degrees;
    m_hasLast[i] = true;
  }
  if (m_answeringCount == 0) {
    return std::nullopt;
  }
  if (changeCount > 0) {
    m_totalDegrees += changeSum / static_cast<double>(changeCount);
  }
  return m_totalDegrees;
}

bool MotorGroup::reset() {
  m_totalDegrees = 0.0;
  m_hasLast.fill(false);
  return true;
}

void MotorGroup::registerPorts(PortRegistry& registry, const char* role,
                               Requirement requirement) const {
  for (std::size_t i = 0; i < m_count; ++i) {
    registry.add(m_motors[i].port, DeviceKind::Motor, role, requirement);
  }
}

void MotorGroup::moveMillivolts(std::int32_t millivolts) {
  for (std::size_t i = 0; i < m_count; ++i) {
    pros::c::motor_move_voltage(signedPort(i), millivolts);
  }
}

void MotorGroup::stop(BrakeMode mode) {
  for (std::size_t i = 0; i < m_count; ++i) {
    pros::c::motor_set_brake_mode(signedPort(i), toBrakeMode(mode));
    pros::c::motor_brake(signedPort(i));
  }
}

std::optional<double> MotorGroup::currentAmps(std::size_t index) const {
  if (index >= m_count) {
    return std::nullopt;
  }
  const std::int32_t milliamps = pros::c::motor_get_current_draw(signedPort(index));
  if (milliamps == PROS_ERR) {
    return std::nullopt;
  }
  return milliamps / 1000.0;
}

std::optional<double> MotorGroup::temperatureCelsius(std::size_t index) const {
  if (index >= m_count) {
    return std::nullopt;
  }
  const double celsius = pros::c::motor_get_temperature(signedPort(index));
  if (!std::isfinite(celsius)) {
    return std::nullopt;
  }
  return celsius;
}

std::optional<double> MotorGroup::velocityRpm(std::size_t index) const {
  if (index >= m_count) {
    return std::nullopt;
  }
  const double rpm = pros::c::motor_get_actual_velocity(signedPort(index));
  if (!std::isfinite(rpm)) {
    return std::nullopt;
  }
  return rpm;
}

}  // namespace cardLib
