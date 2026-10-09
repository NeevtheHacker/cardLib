// driver_control.cpp
// Why: the opcontrol loop: sticks to the chassis, buttons to mechanisms (SPEC section 10).
// Owns: the driver-control loop. The chassis applies the drive through its motion task.
#include "driver/driver_control.hpp"

#include <cstdint>

#include "driver/button_map.hpp"
#include "mechanisms/in_out_mechanism.hpp"
#include "mechanisms/pneumatics.hpp"
#include "robot/chassis_setup.hpp"
#include "robot/devices.hpp"
#include "screen/screen_lines.hpp"

namespace {

constexpr std::uint32_t DRIVER_LOOP_MS = 10;
constexpr std::uint32_t SCREEN_EVERY_LOOPS = 10;  // screen lines at 10 Hz

InOutMechanism::Mode intakeModeFromButtons() {
  if (master.get_digital(INTAKE_BUTTON)) {
    return InOutMechanism::Mode::Intake;
  }
  if (master.get_digital(OUTTAKE_BUTTON)) {
    return InOutMechanism::Mode::Outtake;
  }
  if (master.get_digital(LOW_STORAGE_BUTTON)) {
    return InOutMechanism::Mode::InLowStorage;
  }
  return InOutMechanism::Mode::Off;
}

void toggleHoldBrake() {
  const bool holding = chassis.driverBrake() == cardLib::BrakeMode::Hold;
  chassis.setDriverBrake(holding ? cardLib::BrakeMode::Coast : cardLib::BrakeMode::Hold);
}

}  // namespace

void driverControl() {
  std::uint32_t loop = 0;
  while (true) {
    chassis.arcade(master.get_analog(FORWARD_STICK), master.get_analog(TURN_STICK));

    inOutMech.setMode(intakeModeFromButtons());
    if (master.get_digital_new_press(CLAMP_BUTTON)) {
      clamp.toggle();
    }
    if (master.get_digital_new_press(HOLD_BRAKE_BUTTON)) {
      toggleHoldBrake();
    }

    testModes.runPending();
    if (++loop % SCREEN_EVERY_LOOPS == 0) {
      showScreenLines();
    }
    pros::delay(DRIVER_LOOP_MS);
  }
}
