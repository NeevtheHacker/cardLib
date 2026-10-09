// left_auton.cpp
// Why: left-side match auton. Scores the middle goal, then loads from the match loader.
// Owns: leftAuton(). An example; replace the steps with your own.
#include "autons/auton_list.hpp"
#include "mechanisms/in_out_mechanism.hpp"
#include "robot/chassis_setup.hpp"

void leftAuton() {
  chassis.setPose(0, 0, 0);
  inOutMech.setMode(InOutMechanism::Mode::InLowStorage);
  chassis.turn(74);
  chassis.moveToPose(29, 8, 74, {.maxSpeed = 50});
  chassis.turn(140);
  inOutMech.setMode(InOutMechanism::Mode::Off);
}
