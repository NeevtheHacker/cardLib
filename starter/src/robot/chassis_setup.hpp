// chassis_setup.hpp
// Why: the chassis and cardLib's services, declared once for autons, driver code and the screen.
// Owns: nothing; chassis_setup.cpp defines them.
#pragma once

#include "cardLib/api.hpp"

extern cardLib::Chassis chassis;
extern cardLib::TestModes testModes;
extern cardLib::BlackBox blackBox;
extern cardLib::Display display;
extern cardLib::TerminalLog terminalLog;
extern cardLib::CalibrationRoutines calibrationRoutines;

// Loads calibration, validates every device, calibrates the IMU and starts cardLib's tasks.
// Call once from initialize(), with the robot still.
void setupRobot();
