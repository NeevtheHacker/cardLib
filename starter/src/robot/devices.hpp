// devices.hpp
// Why: every motor, sensor and piston on the robot, declared once and shared by all files.
// Owns: nothing; devices.cpp defines them.
#pragma once

#include "cardLib/api.hpp"
#include "pros/misc.hpp"

extern cardLib::MotorGroup leftDrive;
extern cardLib::MotorGroup rightDrive;
extern cardLib::MotorGroup intakeMotors;

extern cardLib::Imu imu;
extern cardLib::RotationSensor verticalTrackingSensor;
extern cardLib::RotationSensor horizontalTrackingSensor;

extern cardLib::AdiDigitalOut clampPiston;

extern pros::Controller master;

// Where each positional sensor sits: x to the robot's right, y forward, from its center.
extern const cardLib::TrackingWheelConfig verticalTrackingWheel;
extern const cardLib::TrackingWheelConfig horizontalTrackingWheel;
extern const cardLib::ImuConfig imuConfig;
