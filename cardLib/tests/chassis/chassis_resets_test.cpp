// chassis_resets_test.cpp
// Why: proves resetFromWall and resetFromGps end to end on fake hardware: the pose they set,
//      that a refused reset leaves the pose alone, and the chassis-level refusals (SPEC 21.2,
//      21.3).
// Owns: tests for the reset half of chassis/chassis, run with tests/fakes.
#include "cardLib/chassis/chassis.hpp"
#include "doctest.h"
#include "fakes/fake_devices.hpp"
#include "fakes/fake_robot.hpp"
#include "pros/rtos.h"

using namespace cardLib;

namespace {
constexpr std::uint32_t SETTLE_MS = 60;
constexpr std::uint32_t GPS_SETTLE_MS = 150;  // the GPS is read at 10 Hz
using Robot = fake::XDriveRobot;
}  // namespace

TEST_CASE("resets before initialize() and without the sensor are refused") {
  fake::reset();
  Robot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  CHECK(chassis.resetFromWall(0, Wall::Left) == ResetResult::NotRunning);
  CHECK(chassis.resetFromGps() == ResetResult::NotRunning);

  REQUIRE(chassis.initialize().state == ReadyState::Ready);
  CHECK(chassis.resetFromWall(1, Wall::Left) == ResetResult::NoSensor);
  CHECK(chassis.resetFromWall(MAX_DISTANCE_SENSORS, Wall::Left) == ResetResult::NoSensor);
}

TEST_CASE("resetFromWall: the left sensor sets x only; a big change is refused") {
  fake::reset();
  Robot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  REQUIRE(chassis.initialize().state == ReadyState::Ready);
  REQUIRE(chassis.setPose(-50.0, 10.0, 0.0));

  // Robot at x = -50, sensor 6 in to its left at -56, wall at -72: 16 in. Reads 16.5.
  fake::setDistanceInches(Robot::DISTANCE_PORT, 16.5);
  pros::c::delay(SETTLE_MS);
  REQUIRE(chassis.resetFromWall(0, Wall::Left) == ResetResult::Applied);
  pros::c::delay(SETTLE_MS);
  CHECK(chassis.getPose().xInches == doctest::Approx(-49.5).epsilon(0.001));
  CHECK(chassis.getPose().yInches == doctest::Approx(10.0).epsilon(0.001));
  CHECK(chassis.getPose().headingDegrees == doctest::Approx(0.0));

  fake::setDistanceInches(Robot::DISTANCE_PORT, 30.0);  // a 13.5 in change
  pros::c::delay(SETTLE_MS);
  CHECK(chassis.resetFromWall(0, Wall::Left) == ResetResult::ChangeTooLarge);
  CHECK(chassis.getPose().xInches == doctest::Approx(-49.5).epsilon(0.001));

  fake::setDistanceInches(Robot::DISTANCE_PORT, std::nullopt);
  pros::c::delay(SETTLE_MS);
  CHECK(chassis.resetFromWall(0, Wall::Left) == ResetResult::NoReading);
}

TEST_CASE("resetFromWall refuses a sensor that isn't facing the wall") {
  fake::reset();
  Robot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  REQUIRE(chassis.initialize().state == ReadyState::Ready);
  fake::setDistanceInches(Robot::DISTANCE_PORT, 16.0);
  pros::c::delay(SETTLE_MS);
  CHECK(chassis.resetFromWall(0, Wall::Front) == ResetResult::NotSquare);
}

TEST_CASE("resetFromGps: sets x and y, keeps the heading unless asked, refuses a poor fix") {
  fake::reset();
  Robot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  REQUIRE(chassis.initialize().state == ReadyState::Ready);

  fake::setGpsSample(Robot::GPS_PORT, GpsSample{.xInches = 10.0, .yInches = -20.0,
                                                .headingDegrees = 5.0, .errorInches = 0.5});
  pros::c::delay(GPS_SETTLE_MS);
  REQUIRE(chassis.resetFromGps() == ResetResult::Applied);
  pros::c::delay(SETTLE_MS);
  CHECK(chassis.getPose().xInches == doctest::Approx(10.0).epsilon(0.001));
  CHECK(chassis.getPose().yInches == doctest::Approx(-20.0).epsilon(0.001));
  CHECK(chassis.getPose().headingDegrees == doctest::Approx(0.0));

  REQUIRE(chassis.resetFromGps({.resetHeading = true}) == ResetResult::Applied);
  pros::c::delay(SETTLE_MS);
  CHECK(chassis.getPose().headingDegrees == doctest::Approx(5.0));

  fake::setGpsSample(Robot::GPS_PORT, GpsSample{.xInches = 30.0, .yInches = 30.0,
                                                .headingDegrees = 5.0, .errorInches = 3.0});
  pros::c::delay(GPS_SETTLE_MS);
  CHECK(chassis.resetFromGps() == ResetResult::ErrorTooHigh);
  CHECK(chassis.getPose().xInches == doctest::Approx(10.0).epsilon(0.001));

  fake::setGpsSample(Robot::GPS_PORT, std::nullopt);
  pros::c::delay(GPS_SETTLE_MS);
  CHECK(chassis.resetFromGps() == ResetResult::NoReading);
}

TEST_CASE("resets refuse while the robot is moving") {
  fake::reset();
  Robot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  REQUIRE(chassis.initialize().state == ReadyState::Ready);
  fake::setDistanceInches(Robot::DISTANCE_PORT, 16.0);
  fake::setGpsSample(Robot::GPS_PORT, GpsSample{.errorInches = 0.5});
  // Every corner wheel rolling forward 0.5 in per loop: about 35 in/s forward.
  for (int loop = 1; loop <= 20; ++loop) {
    const double degrees = loop * 0.5 / (3.25 * 3.14159265) * 360.0;
    for (const std::uint8_t port : {Robot::FRONT_LEFT, Robot::FRONT_RIGHT, Robot::BACK_LEFT,
                                    Robot::BACK_RIGHT}) {
      fake::setMotorDegrees(port, degrees);
    }
    pros::c::delay(10);
  }
  CHECK(chassis.resetFromWall(0, Wall::Left) == ResetResult::NotStopped);
  CHECK(chassis.resetFromGps() == ResetResult::NotStopped);
}
