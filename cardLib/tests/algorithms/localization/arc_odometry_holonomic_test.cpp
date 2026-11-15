// arc_odometry_holonomic_test.cpp
// Why: proves the SPEC 20.12 odometry changes: sideways velocity, and on a holonomic drive the
//      drive-encoder fallback through least-squares forward kinematics.
// Owns: tests for the holonomic parts of algorithms/localization/arc_odometry.
#include <array>
#include <cmath>

#include "cardLib/algorithms/control/drive_kinematics.hpp"
#include "cardLib/algorithms/localization/arc_odometry.hpp"
#include "cardLib/algorithms/math/angle.hpp"
#include "doctest.h"

using namespace cardLib;

namespace {

// An X-drive, wheels at the corners of a 14 in square (example values).
constexpr DriveKinematics X_DRIVE{{-7.0, 7.0, 45.0, 60.0},
                                  {7.0, 7.0, -45.0, 60.0},
                                  {-7.0, -7.0, -45.0, 60.0},
                                  {7.0, -7.0, 45.0, 60.0}};

// Each wheel's travel for a robot motion (SPEC 20.5).
SensorFrame driveFrame(double right, double forward, double turnRadians) {
  SensorFrame frame;
  frame.driveWheelCount = X_DRIVE.wheelCount();
  for (std::size_t i = 0; i < X_DRIVE.wheelCount(); ++i) {
    const WheelRow& row = X_DRIVE.surfaceRow(i);
    frame.driveWheels[i] = {row.strafe * right + row.forward * forward + row.turn * turnRadians,
                            true};
  }
  return frame;
}

ArcOdometryConfig xDriveOdometry(bool horizontalWheel) {
  return {.hasHorizontalWheel = horizontalWheel, .hasImu = true, .driveKinematics = X_DRIVE};
}

}  // namespace

TEST_CASE("sideways velocity: 0.12 in right per 10 ms settles at 12 in/s") {
  ArcOdometry odometry({.hasHorizontalWheel = true, .hasImu = true});
  RobotState state;
  for (int i = 0; i < 100; ++i) {
    SensorFrame frame;
    frame.imuReadOk = true;
    frame.horizontalWheel = {0.12, true};
    state = odometry.update(frame);
  }
  CHECK(state.sidewaysVelocityInchesPerSecond == doctest::Approx(12.0));
  CHECK(state.pose.xInches == doctest::Approx(12.0));
}

TEST_CASE("holonomic drive without a horizontal wheel: sideways from the drive, degraded") {
  ArcOdometry odometry(xDriveOdometry(false));
  RobotState state;
  for (int i = 0; i < 10; ++i) {
    SensorFrame frame = driveFrame(1.0, 0.5, 0.0);
    frame.imuReadOk = true;
    state = odometry.update(frame);
  }
  CHECK(state.pose.xInches == doctest::Approx(10.0));
  CHECK(state.pose.yInches == doctest::Approx(5.0));
  CHECK(state.health == OdometryHealth::Degraded);
}

TEST_CASE("holonomic drive with a working horizontal wheel trusts the wheel and is OK") {
  ArcOdometry odometry(xDriveOdometry(true));
  SensorFrame frame = driveFrame(1.0, 0.0, 0.0);
  frame.imuReadOk = true;
  frame.horizontalWheel = {0.8, true};  // the tracking wheel disagrees with the drive (slip)
  const RobotState state = odometry.update(frame);
  CHECK(state.pose.xInches == doctest::Approx(0.8));
  CHECK(state.health == OdometryHealth::Ok);
}

TEST_CASE("holonomic drive, faulted IMU: heading from forward kinematics, degraded") {
  ArcOdometry odometry(xDriveOdometry(true));
  SensorFrame frame = driveFrame(0.0, 0.0, degreesToRadians(5.0));
  frame.imuFaulted = true;
  frame.horizontalWheel = {0.0, true};
  const RobotState state = odometry.update(frame);
  CHECK(state.pose.headingDegrees == doctest::Approx(5.0));
  CHECK(state.health == OdometryHealth::Degraded);
}

TEST_CASE("holonomic drive, no tracking wheels: forward from the drive is a normal setup") {
  ArcOdometry odometry({.hasImu = true, .driveKinematics = X_DRIVE});
  SensorFrame frame = driveFrame(0.0, 2.0, 0.0);
  frame.imuReadOk = true;
  const RobotState state = odometry.update(frame);
  CHECK(state.pose.yInches == doctest::Approx(2.0));
  // No horizontal wheel on a holonomic drive is degraded (SPEC 20.12).
  CHECK(state.health == OdometryHealth::Degraded);
}

TEST_CASE("a tank's kinematics leave the tank odometry exactly as it was") {
  ArcOdometry withKinematics({.hasImu = true,
                              .effectiveTrackWidthInches = 12.0,
                              .driveKinematics = tankKinematics(12.0, 60.0)});
  ArcOdometry without({.hasImu = true, .effectiveTrackWidthInches = 12.0});
  SensorFrame frame;
  frame.imuReadOk = true;
  frame.leftDrive = {1.0, true};
  frame.rightDrive = {1.0, true};
  const RobotState a = withKinematics.update(frame);
  const RobotState b = without.update(frame);
  CHECK(a.pose.yInches == b.pose.yInches);
  CHECK(a.health == b.health);
}

TEST_CASE("holonomic drive: a slipping wheel raises driveSlipSuspect; agreeing wheels don't") {
  ArcOdometry odometry(xDriveOdometry(true));
  RobotState state;
  for (int i = 0; i < 20; ++i) {
    SensorFrame frame = driveFrame(0.0, 0.5, 0.0);
    frame.imuReadOk = true;
    frame.horizontalWheel = {0.0, true};
    state = odometry.update(frame);
  }
  CHECK_FALSE(state.driveSlipSuspect);

  // Front left spins 0.4 in a loop more than the others allow: residual 0.1 in on an X-drive.
  for (int i = 0; i < 10; ++i) {
    SensorFrame frame = driveFrame(0.0, 0.5, 0.0);
    frame.driveWheels[0].inches += 0.4;
    frame.imuReadOk = true;
    frame.horizontalWheel = {0.0, true};
    state = odometry.update(frame);
  }
  CHECK(state.driveSlipSuspect);

  odometry.reset({});
  SensorFrame frame = driveFrame(0.0, 0.5, 0.0);
  frame.imuReadOk = true;
  frame.horizontalWheel = {0.0, true};
  CHECK_FALSE(odometry.update(frame).driveSlipSuspect);
}

TEST_CASE("a tank never raises driveSlipSuspect: two wheels always fit exactly") {
  ArcOdometry odometry(
      {.hasImu = true,
       .effectiveTrackWidthInches = 12.0,
       .driveKinematics = DriveKinematics{{-6.0, 0.0, 0.0, 60.0}, {6.0, 0.0, 0.0, 60.0}}});
  RobotState state;
  for (int i = 0; i < 20; ++i) {
    SensorFrame frame;
    frame.imuReadOk = true;
    frame.leftDrive = {0.9, true};
    frame.rightDrive = {0.1, true};
    frame.driveWheelCount = 2;
    frame.driveWheels[0] = {0.9, true};
    frame.driveWheels[1] = {0.1, true};
    state = odometry.update(frame);
  }
  CHECK_FALSE(state.driveSlipSuspect);
}
