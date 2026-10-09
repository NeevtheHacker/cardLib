// screen_lines.cpp
// Why: what each LLEMU line on the User page shows, and the controller's top line.
// Owns: the robot-specific screen content. The display task does all formatting and drawing.
#include "screen/screen_lines.hpp"

#include "mechanisms/in_out_mechanism.hpp"
#include "mechanisms/pneumatics.hpp"
#include "pros/misc.hpp"
#include "robot/chassis_setup.hpp"

void showScreenLines() {
  display.set(0, "intake mode", static_cast<double>(inOutMech.mode()));
  display.set(1, "clamp extended", clamp.isExtended() ? 1.0 : 0.0);
  display.set(2, "battery %", pros::battery::get_capacity());
  display.setControllerLine(0, chassis.driverBrake() == cardLib::BrakeMode::Hold ? "HOLD" : "");
}
