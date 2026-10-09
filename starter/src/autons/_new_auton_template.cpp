// _new_auton_template.cpp
// Why: the file to copy when adding an auton run. Copy it, rename the file and the function,
//      write the steps, declare it in auton_list.hpp and add one line to AUTONS.
// Owns: newAutonTemplate(), which isn't in the selector.
#include "autons/auton_list.hpp"
#include "robot/chassis_setup.hpp"

void newAutonTemplate() {
  // 1. Tell odometry where the robot starts: x, y in inches, heading in degrees (0 = +y).
  chassis.setPose(0, 0, 0);

  // 2. The steps. Every motion returns a MotionResult (Settled, Stalled, TimedOut, ...).
  chassis.move(24);
  chassis.turn(90);
}
