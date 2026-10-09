// in_out_mechanism.cpp
// Why: maps each intake mode to a motor output.
// Owns: inOutMech, the robot's intake.
#include "mechanisms/in_out_mechanism.hpp"

#include "robot/devices.hpp"

InOutMechanism inOutMech(intakeMotors);

void InOutMechanism::setMode(Mode mode) {
  m_mode = mode;
  update();
}

void InOutMechanism::update() {
  switch (m_mode) {
    case Mode::Off:
      m_motors.stop(cardLib::BrakeMode::Coast);
      break;
    case Mode::Intake:
      m_motors.moveMillivolts(INTAKE_MILLIVOLTS);
      break;
    case Mode::Outtake:
      m_motors.moveMillivolts(OUTTAKE_MILLIVOLTS);
      break;
    case Mode::InLowStorage:
      m_motors.moveMillivolts(LOW_STORAGE_MILLIVOLTS);
      break;
  }
}
