// driver_control.hpp
// Why: the driver-control loop, kept out of main.cpp.
// Owns: nothing; driver_control.cpp holds the loop.
#pragma once

// Runs until the match ends: drive, mechanisms, test modes, screen lines.
void driverControl();
