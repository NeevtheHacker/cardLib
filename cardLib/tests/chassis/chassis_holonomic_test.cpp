// chassis_holonomic_test.cpp
// Why: proves Chassis::holonomic() end to end on fake hardware: the SPEC 20.3 wheel pattern,
//      field-centric rotation by the IMU heading, resetFieldHeading, the drive mode, and the
//      tank cases (SPEC 20.10, 20.11).
// Owns: tests for the driver-control half of chassis/chassis, run with tests/fakes.
#include "cardLib/chassis/chassis.hpp"
#include "doctest.h"
#include "fakes/fake_devices.hpp"
#include "fakes/fake_robot.hpp"
#include "pros/rtos.h"

using namespace cardLib;

namespace {

constexpr std::uint32_t SETTLE_MS = 60;  // a few loops of both tasks

using Robot = fake::XDriveRobot;

struct WheelSigns {
  int frontLeft;
  int frontRight;
  int backLeft;
  int backRight;
};

int sign(std::int32_t millivolts) { return millivolts > 0 ? 1 : (millivolts < 0 ? -1 : 0); }

WheelSigns wheelSigns() {
  return {sign(fake::motorMillivolts(Robot::FRONT_LEFT)),
          sign(fake::motorMillivolts(Robot::FRONT_RIGHT)),
          sign(fake::motorMillivolts(Robot::BACK_LEFT)),
          sign(fake::motorMillivolts(Robot::BACK_RIGHT))};
}

void checkSigns(const WheelSigns& actual, const WheelSigns& expected) {
  CHECK(actual.frontLeft == expected.frontLeft);
  CHECK(actual.frontRight == expected.frontRight);
  CHECK(actual.backLeft == expected.backLeft);
  CHECK(actual.backRight == expected.backRight);
}

constexpr WheelSigns FORWARD{1, 1, 1, 1};
constexpr WheelSigns STRAFE_RIGHT{1, -1, -1, 1};
constexpr WheelSigns STRAFE_LEFT{-1, 1, 1, -1};
constexpr WheelSigns CLOCKWISE{1, -1, 1, -1};

}  // namespace

TEST_CASE("holonomic(): each stick drives the SPEC 20.3 pattern on an X-drive") {
  fake::reset();
  Robot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  REQUIRE(chassis.initialize().state == ReadyState::Ready);

  chassis.holonomic(100.0, 0.0, 0.0);
  pros::c::delay(SETTLE_MS);
  checkSigns(wheelSigns(), FORWARD);

  chassis.holonomic(0.0, 100.0, 0.0);
  pros::c::delay(SETTLE_MS);
  checkSigns(wheelSigns(), STRAFE_RIGHT);

  chassis.holonomic(0.0, 0.0, 100.0);
  pros::c::delay(SETTLE_MS);
  checkSigns(wheelSigns(), CLOCKWISE);
  CHECK_FALSE(chassis.fieldCentricIgnored());
}

TEST_CASE("field-centric: facing right, stick forward slides the robot to its left") {
  fake::reset();
  Robot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  REQUIRE(chassis.initialize().state == ReadyState::Ready);
  pros::c::delay(SETTLE_MS);  // odometry takes its IMU baseline on its first loop
  fake::setImuRotation(90.0);
  pros::c::delay(SETTLE_MS);
  REQUIRE(chassis.getPose().headingDegrees == doctest::Approx(90.0));

  SUBCASE("per-call option, robot-centric mode") {
    chassis.holonomic(100.0, 0.0, 0.0, {.fieldCentric = true});
    pros::c::delay(SETTLE_MS);
    checkSigns(wheelSigns(), STRAFE_LEFT);
    chassis.holonomic(100.0, 0.0, 0.0);
    pros::c::delay(SETTLE_MS);
    checkSigns(wheelSigns(), FORWARD);
  }
  SUBCASE("field-centric mode, and a per-call override back to robot-centric") {
    chassis.setDriveMode(DriveMode::FieldCentric);
    chassis.holonomic(100.0, 0.0, 0.0);
    pros::c::delay(SETTLE_MS);
    checkSigns(wheelSigns(), STRAFE_LEFT);
    chassis.holonomic(100.0, 0.0, 0.0, {.fieldCentric = false});
    pros::c::delay(SETTLE_MS);
    checkSigns(wheelSigns(), FORWARD);
  }
  SUBCASE("resetFieldHeading makes the current facing forward") {
    chassis.resetFieldHeading();
    chassis.holonomic(100.0, 0.0, 0.0, {.fieldCentric = true});
    pros::c::delay(SETTLE_MS);
    checkSigns(wheelSigns(), FORWARD);
  }
}

TEST_CASE("field-centric falls back to robot-centric once the IMU faults") {
  fake::reset();
  Robot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  REQUIRE(chassis.initialize().state == ReadyState::Ready);
  pros::c::delay(SETTLE_MS);  // odometry takes its IMU baseline on its first loop
  fake::setImuRotation(90.0);
  pros::c::delay(SETTLE_MS);
  fake::setImuPresent(false);
  pros::c::delay(SETTLE_MS);  // three failed reads in a row fault it
  chassis.holonomic(100.0, 0.0, 0.0, {.fieldCentric = true});
  pros::c::delay(SETTLE_MS);
  checkSigns(wheelSigns(), FORWARD);
}

TEST_CASE("on a tank, field-centric is ignored and reported, and strafe is Unsupported") {
  fake::reset();
  fake::TankRobot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  REQUIRE(chassis.initialize().state == ReadyState::Ready);
  CHECK_FALSE(chassis.fieldCentricIgnored());

  chassis.holonomic(100.0, 0.0, 0.0, {.fieldCentric = true});
  pros::c::delay(SETTLE_MS);
  CHECK(chassis.fieldCentricIgnored());
  CHECK(fake::motorMillivolts(5) > 0);
  CHECK(fake::motorMillivolts(6) > 0);
  CHECK(chassis.strafe(12.0) == MotionResult::Unsupported);
  CHECK(chassis.pivotTurn(90.0, LocalOffset{0.0, 6.0}) == MotionResult::Unsupported);
}

TEST_CASE("setDriveMode(FieldCentric) before initialize() on a tank is reported by initialize()") {
  fake::reset();
  fake::TankRobot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  chassis.setDriveMode(DriveMode::FieldCentric);
  CHECK_FALSE(chassis.fieldCentricIgnored());
  REQUIRE(chassis.initialize().state == ReadyState::Ready);
  CHECK(chassis.fieldCentricIgnored());
}

TEST_CASE("field-centric set before initialize() on a drive that strafes is not flagged") {
  fake::reset();
  Robot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  chassis.setDriveMode(DriveMode::FieldCentric);
  REQUIRE(chassis.initialize().state == ReadyState::Ready);
  CHECK_FALSE(chassis.fieldCentricIgnored());
}

TEST_CASE("setDriveMode(FieldCentric) on a tank is reported at once") {
  fake::reset();
  fake::TankRobot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  REQUIRE(chassis.initialize().state == ReadyState::Ready);
  chassis.setDriveMode(DriveMode::FieldCentric);
  CHECK(chassis.fieldCentricIgnored());
  CHECK(chassis.driveMode() == DriveMode::FieldCentric);
}
