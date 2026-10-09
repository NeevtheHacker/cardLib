// devices.cpp
// Why: the physical robot description: ports, directions, cartridges, sensor positions.
// Owns: every device object. Change these to match your robot (ports are in ports.hpp).
#include "robot/devices.hpp"

#include "robot/ports.hpp"

using cardLib::Cartridge;

cardLib::MotorGroup leftDrive(
    {
        {.port = LEFT_FRONT_PORT, .reversed = true, .cartridge = Cartridge::Blue600},
        {.port = LEFT_MIDDLE_PORT, .reversed = true, .cartridge = Cartridge::Blue600},
        {.port = LEFT_BACK_PORT, .reversed = true, .cartridge = Cartridge::Blue600},
    },
    "left drive");

cardLib::MotorGroup rightDrive(
    {
        {.port = RIGHT_FRONT_PORT, .reversed = false, .cartridge = Cartridge::Blue600},
        {.port = RIGHT_MIDDLE_PORT, .reversed = false, .cartridge = Cartridge::Blue600},
        {.port = RIGHT_BACK_PORT, .reversed = false, .cartridge = Cartridge::Blue600},
    },
    "right drive");

cardLib::MotorGroup intakeMotors({{.port = INTAKE_PORT, .cartridge = Cartridge::Blue600}},
                                 "intake");

cardLib::Imu imu(IMU_PORT);
cardLib::RotationSensor verticalTrackingSensor(VERTICAL_TRACKING_PORT, false);
cardLib::RotationSensor horizontalTrackingSensor(HORIZONTAL_TRACKING_PORT, false);

cardLib::AdiDigitalOut clampPiston(CLAMP_PISTON_PORT, false);

pros::Controller master(pros::E_CONTROLLER_MASTER);

const cardLib::TrackingWheelConfig verticalTrackingWheel{
    .sensor = &verticalTrackingSensor, .diameterInches = 2.75, .xInches = 0.0, .yInches = 0.0};
const cardLib::TrackingWheelConfig horizontalTrackingWheel{
    .sensor = &horizontalTrackingSensor, .diameterInches = 2.75, .xInches = 0.0, .yInches = -1.5};
const cardLib::ImuConfig imuConfig{.sensor = &imu, .orientation = cardLib::ImuOrientation::Flat};
