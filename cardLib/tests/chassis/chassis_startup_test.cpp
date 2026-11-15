// chassis_startup_test.cpp
// Why: proves the drive problems startup validation catches on fake hardware: a wheel module
//      with no motor group, and a config with no drive (SPEC section 3, 20.14).
// Owns: tests for the drive checks in Chassis::initialize(), run with tests/fakes.
#include "cardLib/chassis/chassis.hpp"
#include "cardLib/chassis/sensor_status.hpp"
#include "doctest.h"
#include "fakes/fake_devices.hpp"
#include "fakes/fake_robot.hpp"

using namespace cardLib;

TEST_CASE("a wheel module with no motor group is a startup fault, not skipped silently") {
  fake::reset();
  fake::XDriveRobot robot;
  ChassisConfig config = robot.config();
  config.drive.modules[2].motors = nullptr;
  Chassis chassis(config);
  fake::TaskGuard guard;
  const StartupReport& report = chassis.initialize();
  CHECK(report.modulesWithoutMotors == 1);
  CHECK(report.state == ReadyState::SensorFault);
  const SensorStatusList sensors = sensorStatus(config, report, chassis.odometrySnapshot());
  CHECK(sensors.lines[2].status == SensorStatus::Missing);  // the "drive" line
}

TEST_CASE("every module with motors: no fault") {
  fake::reset();
  fake::XDriveRobot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  const StartupReport& report = chassis.initialize();
  CHECK(report.modulesWithoutMotors == 0);
  CHECK_FALSE(report.noDrive);
  CHECK(report.state == ReadyState::Ready);
}

TEST_CASE("the left/right shorthand needs Geometry; without it there is no drive") {
  fake::reset();
  fake::TankRobot robot;
  ChassisConfig config;
  config.drive.left = &robot.left;
  config.drive.right = &robot.right;
  config.sensors.imu = &robot.imuConfig;

  SUBCASE("without geometry") {
    Chassis chassis(config);
    fake::TaskGuard guard;
    const StartupReport& report = chassis.initialize();
    CHECK(report.noDrive);
    CHECK(report.state == ReadyState::SensorFault);
  }
  SUBCASE("with geometry") {
    config.geometry = Geometry{.wheelDiameterInches = 3.25, .trackWidthInches = 12.0};
    Chassis chassis(config);
    fake::TaskGuard guard;
    const StartupReport& report = chassis.initialize();
    CHECK_FALSE(report.noDrive);
    CHECK(report.state == ReadyState::Ready);
    CHECK(chassis.drive().moduleCount == 2);
    CHECK(chassis.drive().modules[0].xInches == -6.0);
  }
}
