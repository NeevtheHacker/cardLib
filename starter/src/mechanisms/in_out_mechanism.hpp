// in_out_mechanism.hpp
// Why: the intake/outtake as a few named modes, so autons and driver code say what they want
//      ("store low"), not motor voltages.
// Owns: the intake motors (only writer) and the current mode. Change the modes for your robot.
#pragma once

#include <cstdint>

#include "cardLib/api.hpp"

class InOutMechanism {
 public:
  enum class Mode { Off, Intake, Outtake, InLowStorage };

  explicit InOutMechanism(cardLib::MotorGroup& motors) : m_motors(motors) {}

  void setMode(Mode mode);
  Mode mode() const { return m_mode; }

  // Re-applies the current mode. Call every driver-control loop.
  void update();

 private:
  static constexpr std::int32_t INTAKE_MILLIVOLTS = 12000;
  static constexpr std::int32_t OUTTAKE_MILLIVOLTS = -12000;
  static constexpr std::int32_t LOW_STORAGE_MILLIVOLTS = 6000;

  cardLib::MotorGroup& m_motors;
  Mode m_mode = Mode::Off;
};

extern InOutMechanism inOutMech;
