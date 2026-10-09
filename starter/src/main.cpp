// main.cpp
// Why: the PROS lifecycle, and nothing else: each mode hands off to its own file.
// Owns: initialize, disabled, competition_initialize, autonomous, opcontrol.
#include "main.h"

#include "driver/driver_control.hpp"
#include "robot/chassis_setup.hpp"

// Runs once at startup with the robot still: validation and IMU calibration take a few seconds.
void initialize() { setupRobot(); }

void disabled() {}

// Choose the auton on the brain screen (Auton and Auton options pages) while disabled.
void competition_initialize() {}

void autonomous() { display.runSelectedAuton(); }

void opcontrol() { driverControl(); }
