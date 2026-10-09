// motor_group.hpp
// Why: a fixed-size group of motors that keeps one shaft angle even when a motor drops out.
// Owns: its motors' configs, per-motor health and the group's cumulative angle.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>

#include "cardLib/hardware/encoder.hpp"
#include "cardLib/hardware/hardware_config.hpp"
#include "cardLib/hardware/sensor_health.hpp"

namespace cardLib {

constexpr std::size_t MAX_MOTORS_PER_GROUP = 4;

class MotorGroup : public Encoder {
 public:
  // Up to MAX_MOTORS_PER_GROUP motors; initialize() fails if more are given.
  MotorGroup(std::initializer_list<MotorConfig> motors, const char* role = "motor group");

  // Sets encoder units to degrees and each motor's cartridge.
  bool initialize() override;

  // Average change of the motors that answered, added to a running total, so a motor that
  // disconnects, reconnects or is removed never makes the angle jump (SPEC section 17).
  std::optional<double> readDegrees() override;
  bool reset() override;
  void registerPorts(PortRegistry& registry, const char* role,
                     Requirement requirement) const override;

  // -12000 to 12000 mV on every motor. The only way cardLib drives a motor.
  void moveMillivolts(std::int32_t millivolts);
  void stop(BrakeMode mode);

  std::size_t size() const { return m_count; }
  const MotorConfig& motor(std::size_t index) const { return m_motors[index]; }
  const SensorHealth& health(std::size_t index) const { return m_health[index]; }
  const char* role() const { return m_role; }

  // Motors that answered the last readDegrees().
  std::size_t answeringCount() const { return m_answeringCount; }

  std::optional<double> currentAmps(std::size_t index) const;
  std::optional<double> temperatureCelsius(std::size_t index) const;
  std::optional<double> velocityRpm(std::size_t index) const;  // output shaft, signed

 private:
  std::int8_t signedPort(std::size_t index) const;

  std::array<MotorConfig, MAX_MOTORS_PER_GROUP> m_motors = {};
  std::array<SensorHealth, MAX_MOTORS_PER_GROUP> m_health = {};
  std::array<double, MAX_MOTORS_PER_GROUP> m_lastDegrees = {};
  std::array<bool, MAX_MOTORS_PER_GROUP> m_hasLast = {};
  std::size_t m_count = 0;
  bool m_tooManyMotors = false;
  std::size_t m_answeringCount = 0;
  double m_totalDegrees = 0.0;
  const char* m_role;
};

}  // namespace cardLib
