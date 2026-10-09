// right_auton.cpp
// Why: right-side match auton. Takes the goal to the left, then backs out and turns.
// Owns: rightAuton(). An example of reacting to a motion's result and of chaining.
#include "autons/auton_list.hpp"
#include "mechanisms/in_out_mechanism.hpp"
#include "mechanisms/pneumatics.hpp"
#include "robot/chassis_setup.hpp"

void rightAuton() {
  chassis.setPose(0, 0, 0);
  inOutMech.setMode(InOutMechanism::Mode::Intake);

  // Drive to the goal; if something blocks the robot, back off and try once more.
  if (chassis.moveToPoint(-24, 24) == cardLib::MotionResult::Stalled) {
    chassis.move(-6);
    chassis.moveToPoint(-24, 24, {.timeoutMs = 1500});
  }
  clamp.extend();

  // Chained moves: earlyExit plus minSpeed lets the robot flow from one into the next.
  chassis.move(-12, {.minSpeed = 40, .earlyExit = 2});
  chassis.turn(-140);
  inOutMech.setMode(InOutMechanism::Mode::Off);
}
