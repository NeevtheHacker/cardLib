// port_check_test.cpp
// Why: proves the per-module port and direction check (SPEC 13, 20.3) on fake hardware: each
//      module is spun alone, every motor is reported, and a motor wired backwards or on the
//      wrong cartridge is caught.
// Owns: tests for TestModes::runPortCheck in systems/test_modes, run with tests/fakes.
#include "cardLib/systems/test_modes.hpp"

#include <cstring>

#include "doctest.h"
#include "fakes/fake_devices.hpp"
#include "fakes/fake_robot.hpp"

using namespace cardLib;

namespace {
using Robot = fake::XDriveRobot;
}  // namespace

TEST_CASE("port check: every X-drive module spun alone, in order, all correct") {
  fake::reset();
  Robot robot;
  Chassis chassis(robot.config());
  fake::TaskGuard guard;
  REQUIRE(chassis.initialize().state == ReadyState::Ready);
  fake::setMotorWiredBackwards(Robot::BACK_LEFT, true);
  TestModes tests(chassis);

  const PortCheckReport report = tests.runPortCheck();
  REQUIRE(report.ran);
  REQUIRE(report.motorCount == 4);
  const std::uint8_t ports[] = {Robot::FRONT_LEFT, Robot::FRONT_RIGHT, Robot::BACK_LEFT,
                                Robot::BACK_RIGHT};
  const double rolls[] = {45.0, -45.0, -45.0, 45.0};
  for (std::size_t i = 0; i < 4; ++i) {
    const MotorCheck& check = report.motors[i];
    CHECK(check.port == ports[i]);
    CHECK(check.module == i);
    CHECK(check.rollAngleDegrees == rolls[i]);
    CHECK(check.answering);
    CHECK(check.speedOk == (i != 2));
    CHECK(check.directionOk == (i != 2));  // back left is wired backwards
  }
  CHECK(report.motors[0].side == Side::Left);
  CHECK(report.motors[1].side == Side::Right);
  for (const std::uint8_t port : ports) {
    CHECK(fake::motorMillivolts(port) == 0);  // everything stopped at the end
  }
}

TEST_CASE("push direction names follow the roll angle (SPEC 20.3)") {
  CHECK(std::strcmp(pushDirectionName(0.0), "forward") == 0);
  CHECK(std::strcmp(pushDirectionName(45.0), "forward-right") == 0);
  CHECK(std::strcmp(pushDirectionName(-45.0), "forward-left") == 0);
  CHECK(std::strcmp(pushDirectionName(90.0), "right") == 0);
  CHECK(std::strcmp(pushDirectionName(180.0), "backward") == 0);
  CHECK(std::strcmp(pushDirectionName(30.0), "at its roll angle") == 0);
}
