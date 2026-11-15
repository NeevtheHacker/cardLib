// sensor_status_test.cpp
// Why: proves the Sensors page words (SPEC 21.4): each configured sensor is listed once, in
//      order, and shows OK, Suspect, Faulted or Missing from the startup report and the
//      odometry snapshot.
// Owns: tests for chassis/sensor_status (pure; the fake robot only supplies configs).
#include "cardLib/chassis/sensor_status.hpp"

#include <cstring>

#include "doctest.h"
#include "fakes/fake_robot.hpp"

using namespace cardLib;

namespace {

SensorStatus statusOf(const SensorStatusList& list, const char* name) {
  for (std::size_t i = 0; i < list.count; ++i) {
    if (std::strcmp(list.lines[i].name, name) == 0) {
      return list.lines[i].status;
    }
  }
  FAIL("no line named ", name);
  return SensorStatus::Ok;
}

OdometrySnapshot healthySnapshot() {
  OdometrySnapshot odometry;
  odometry.distances[0] = {.inches = 20.0, .confidence = 1.0, .valid = true};
  odometry.gps = {.errorInches = 0.4, .valid = true};
  return odometry;
}

}  // namespace

TEST_CASE("every configured sensor is listed once, in order, and healthy ones are OK") {
  fake::reset();
  fake::XDriveRobot robot;
  const SensorStatusList list = sensorStatus(robot.config(), {}, healthySnapshot());
  REQUIRE(list.count == 5);
  const char* const expected[] = {"IMU", "horiz", "drive", "dist 1", "GPS"};
  for (std::size_t i = 0; i < 5; ++i) {
    CHECK(std::strcmp(list.lines[i].name, expected[i]) == 0);
    CHECK(list.lines[i].status == SensorStatus::Ok);
  }
}

TEST_CASE("sensors a robot doesn't have are left out") {
  fake::reset();
  fake::TankRobot robot;
  const SensorStatusList list = sensorStatus(robot.config(), {}, {});
  REQUIRE(list.count == 2);
  CHECK(std::strcmp(list.lines[0].name, "IMU") == 0);
  CHECK(std::strcmp(list.lines[1].name, "drive") == 0);
}

TEST_CASE("a port problem at startup shows as Missing") {
  fake::reset();
  fake::XDriveRobot robot;
  StartupReport startup;
  startup.portIssues[0] = {fake::XDriveRobot::DISTANCE_PORT, DeviceKind::Distance,
                           "distance sensor", Requirement::Optional, PortProblem::Missing};
  startup.portIssues[1] = {fake::XDriveRobot::FRONT_RIGHT, DeviceKind::Motor, "front right",
                           Requirement::Required, PortProblem::Missing};
  startup.portIssueCount = 2;
  const SensorStatusList list = sensorStatus(robot.config(), startup, healthySnapshot());
  CHECK(statusOf(list, "dist 1") == SensorStatus::Missing);
  CHECK(statusOf(list, "drive") == SensorStatus::Missing);
  CHECK(statusOf(list, "GPS") == SensorStatus::Ok);
}

TEST_CASE("faults mid-run show as Faulted, doubtful readings as Suspect") {
  fake::reset();
  fake::XDriveRobot robot;
  OdometrySnapshot odometry = healthySnapshot();
  odometry.imuFaulted = true;
  odometry.horizontalWheelFaulted = true;
  odometry.gpsFaulted = true;
  odometry.distances[0].valid = false;
  odometry.state.driveSlipSuspect = true;
  const SensorStatusList list = sensorStatus(robot.config(), {}, odometry);
  CHECK(statusOf(list, "IMU") == SensorStatus::Faulted);
  CHECK(statusOf(list, "horiz") == SensorStatus::Faulted);
  CHECK(statusOf(list, "GPS") == SensorStatus::Faulted);
  CHECK(statusOf(list, "dist 1") == SensorStatus::Suspect);
  CHECK(statusOf(list, "drive") == SensorStatus::Suspect);
}

TEST_CASE("a GPS fix worse than 1 in, and a drive motor that stopped answering") {
  fake::reset();
  fake::XDriveRobot robot;
  OdometrySnapshot odometry = healthySnapshot();
  odometry.gps.errorInches = 2.0;
  odometry.driveMotorMissing = true;
  const SensorStatusList list = sensorStatus(robot.config(), {}, odometry);
  CHECK(statusOf(list, "GPS") == SensorStatus::Suspect);
  CHECK(statusOf(list, "drive") == SensorStatus::Faulted);
  CHECK(std::strcmp(sensorStatusName(SensorStatus::Missing), "Missing") == 0);
}
