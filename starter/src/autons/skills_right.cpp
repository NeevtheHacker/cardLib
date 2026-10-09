// skills_right.cpp
// Why: skills run starting on the right. Shows async motions and waitUntil.
// Owns: skillsRight(). An example; replace the steps with your own.
#include "autons/auton_list.hpp"
#include "mechanisms/in_out_mechanism.hpp"
#include "robot/chassis_setup.hpp"

void skillsRight() {
  chassis.setPose(0, 0, 0);

  // Start the intake 12 in into the drive, without stopping the drive.
  chassis.moveToPoint(0, 36, {.async = true});
  chassis.waitUntil(12);
  inOutMech.setMode(InOutMechanism::Mode::Intake);
  chassis.waitUntilDone();

  chassis.turnToPoint(24, 48);
  chassis.moveToPose(24, 48, 45);
  chassis.pivotTurn(90, cardLib::Side::Right);
  inOutMech.setMode(InOutMechanism::Mode::Off);
}
