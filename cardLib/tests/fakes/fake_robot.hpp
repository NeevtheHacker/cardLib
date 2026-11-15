// fake_robot.hpp
// Why: two example robots for the chassis tests, an X-drive with every sensor and a plain tank,
//      built from fake devices. All positions and ports are example values only.
// Owns: the fake robots' devices and configs, which must outlive the Chassis built from them.
#pragma once

#include "cardLib/chassis/chassis.hpp"
#include "cardLib/chassis/drive_presets.hpp"
#include "cardLib/hardware/distance_sensor.hpp"
#include "cardLib/hardware/gps_sensor.hpp"
#include "cardLib/hardware/imu.hpp"
#include "cardLib/hardware/rotation_sensor.hpp"
#include "fakes/fake_devices.hpp"

namespace fake {

// Corner wheels on a 12 in square: FL 1, FR 2, BL 3, BR 4. IMU 10, left distance sensor 11
// (6 in left of center, facing left), GPS 12 (at the center, facing forward), horizontal
// tracking wheel 13.
struct XDriveRobot {
  static constexpr std::uint8_t FRONT_LEFT = 1;
  static constexpr std::uint8_t FRONT_RIGHT = 2;
  static constexpr std::uint8_t BACK_LEFT = 3;
  static constexpr std::uint8_t BACK_RIGHT = 4;
  static constexpr std::uint8_t IMU_PORT = 10;
  static constexpr std::uint8_t DISTANCE_PORT = 11;
  static constexpr std::uint8_t GPS_PORT = 12;
  static constexpr std::uint8_t HORIZONTAL_PORT = 13;

  cardLib::MotorGroup frontLeft{{{.port = FRONT_LEFT}}, "front left"};
  cardLib::MotorGroup frontRight{{{.port = FRONT_RIGHT}}, "front right"};
  cardLib::MotorGroup backLeft{{{.port = BACK_LEFT}}, "back left"};
  cardLib::MotorGroup backRight{{{.port = BACK_RIGHT}}, "back right"};
  cardLib::Imu imu{IMU_PORT};
  cardLib::DistanceSensor distance{DISTANCE_PORT};
  cardLib::GpsSensor gps{GPS_PORT};
  cardLib::RotationSensor horizontal{HORIZONTAL_PORT, false};

  cardLib::ImuConfig imuConfig{.sensor = &imu};
  cardLib::DistanceSensorConfig distanceConfig{
      .sensor = &distance, .xInches = -6.0, .yInches = 0.0, .headingDegrees = -90.0};
  cardLib::GpsConfig gpsConfig{.sensor = &gps};
  cardLib::TrackingWheelConfig horizontalWheel{.sensor = &horizontal,
                                               .diameterInches = cardLib::Omniwheel::NEW_275};

  XDriveRobot() {
    for (const std::uint8_t port : {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT}) {
      plugMotor(port);
    }
    plugImu(IMU_PORT);
    plugDistance(DISTANCE_PORT);
    plugGps(GPS_PORT);
    plugRotation(HORIZONTAL_PORT);
  }

  cardLib::ChassisConfig config() {
    namespace Omniwheel = cardLib::Omniwheel;
    cardLib::ChassisConfig config;
    config.drive = cardLib::customDrive({
        {&frontLeft, Omniwheel::NEW_325, {1, 1}, -6.0, 6.0, 45.0},
        {&frontRight, Omniwheel::NEW_325, {1, 1}, 6.0, 6.0, -45.0},
        {&backLeft, Omniwheel::NEW_325, {1, 1}, -6.0, -6.0, -45.0},
        {&backRight, Omniwheel::NEW_325, {1, 1}, 6.0, -6.0, 45.0},
    });
    config.sensors.imu = &imuConfig;
    config.sensors.horizontalWheel = &horizontalWheel;
    config.sensors.distanceSensors[0] = &distanceConfig;
    config.sensors.gps = &gpsConfig;
    return config;
  }
};

// A two-motor tank on ports 5 (left) and 6 (right), 12 in track, with an IMU on 10.
struct TankRobot {
  cardLib::MotorGroup left{{{.port = 5}}, "left drive"};
  cardLib::MotorGroup right{{{.port = 6}}, "right drive"};
  cardLib::Imu imu{10};
  cardLib::ImuConfig imuConfig{.sensor = &imu};

  TankRobot() {
    plugMotor(5);
    plugMotor(6);
    plugImu(10);
  }

  cardLib::ChassisConfig config() {
    cardLib::ChassisConfig config;
    config.drive = cardLib::tank(left, right, cardLib::Omniwheel::NEW_325, {1, 1}, 12.0);
    config.sensors.imu = &imuConfig;
    return config;
  }
};

}  // namespace fake
