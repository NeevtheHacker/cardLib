// adi_digital.cpp
// Why: implements AdiDigitalOut on the PROS ADI API.
// Owns: nothing beyond AdiDigitalOut's own state.
#include "cardLib/hardware/adi_digital.hpp"

#include "pros/adi.h"
#include "pros/error.h"

namespace cardLib {

bool AdiDigitalOut::initialize() {
  if (pros::c::adi_port_set_config(m_port, pros::E_ADI_DIGITAL_OUT) == PROS_ERR) {
    return false;
  }
  return set(m_state);
}

bool AdiDigitalOut::set(bool state) {
  if (pros::c::adi_digital_write(m_port, state) == PROS_ERR) {
    return false;
  }
  m_state = state;
  return true;
}

}  // namespace cardLib
