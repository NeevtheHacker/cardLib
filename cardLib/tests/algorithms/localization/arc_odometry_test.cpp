// arc_odometry_test.cpp
// Why: proves the SPEC section 5 odometry: spin, forward, strafe, a known arc, wrong offsets
//      caught, the small-angle branch, IMU scale and the section 4 fallbacks.
// Owns: tests for algorithms/localization/arc_odometry.
#include "cardLib/algorithms/localization/arc_odometry.hpp"

#include <cmath>
#include <numbers>

#include "cardLib/algorithms/math/angle.hpp"
#include "doctest.h"

using namespace cardLib;

namespace {

constexpr double SPIN_WHEEL_X = 6.0;
constexpr double SPIN_WHEEL_Y = -1.5;

// IMU + one vertical wheel at x = 6 + horizontal wheel at y = -1.5.
ArcOdometryConfig imuWithTwoWheels(double configuredVerticalX = SPIN_WHEEL_X) {
  return {.verticalWheelCount = 1,
          .verticalWheelXInches = {configuredVerticalX, 0.0},
          .hasHorizontalWheel = true,
          .horizontalWheelYInches = SPIN_WHEEL_Y,
          .hasImu = true,
          .effectiveTrackWidthInches = 12.0};
}

// A frame where every configured sensor reads, the robot spinning stepDegrees clockwise in
// place: each wheel moves only from turning (SPEC section 5 step 3 run backwards).
SensorFrame spinFrame(double imuRotationDegrees, double stepDegrees) {
  const double stepRadians = degreesToRadians(stepDegrees);
  SensorFrame frame;
  frame.imuValid = true;
  frame.imuRotationDegrees = imuRotationDegrees;
  frame.verticalWheels[0] = {-SPIN_WHEEL_X * stepRadians, true};
  frame.horizontalWheel = {SPIN_WHEEL_Y * stepRadians, true};
  frame.leftDrive = {6.0 * stepRadians, true};
  frame.rightDrive = {-6.0 * stepRadians, true};
  return frame;
}

SensorFrame straightFrame(double imuRotationDegrees, double forwardInches,
                          double sidewaysInches = 0.0) {
  SensorFrame frame;
  frame.imuValid = true;
  frame.imuRotationDegrees = imuRotationDegrees;
  frame.verticalWheels[0] = {forwardInches, true};
  frame.horizontalWheel = {sidewaysInches, true};
  frame.leftDrive = {forwardInches, true};
  frame.rightDrive = {forwardInches, true};
  return frame;
}

// Spins 36 steps of 10 degrees and returns the largest distance from the start point.
double maxSpinDrift(ArcOdometry& odometry) {
  odometry.reset({0.0, 0.0, 0.0});
  odometry.update(spinFrame(0.0, 0.0));
  double maxDrift = 0.0;
  for (int step = 1; step <= 36; ++step) {
    const RobotState state = odometry.update(spinFrame(10.0 * step, 10.0));
    maxDrift = std::max(maxDrift, std::hypot(state.pose.xInches, state.pose.yInches));
  }
  return maxDrift;
}

}  // namespace

TEST_CASE("pure spin of 360 degrees leaves x and y unchanged") {
  ArcOdometry odometry(imuWithTwoWheels());
  CHECK(maxSpinDrift(odometry) < 1e-9);
  const RobotState state = odometry.update(spinFrame(360.0, 0.0));
  CHECK(state.pose.headingDegrees == doctest::Approx(360.0));
  CHECK(state.health == OdometryHealth::Ok);
}

TEST_CASE("wrong offsets on purpose make the spin test fail, proving it can catch them") {
  ArcOdometry odometry(imuWithTwoWheels(4.0));
  CHECK(maxSpinDrift(odometry) > 0.5);
}

TEST_CASE("pure 24 in forward at heading 0 adds 24 to y") {
  ArcOdometry odometry(imuWithTwoWheels());
  odometry.reset({0.0, 0.0, 0.0});
  odometry.update(straightFrame(0.0, 0.0));
  const RobotState state = odometry.update(straightFrame(0.0, 24.0));
  CHECK(state.pose.xInches == doctest::Approx(0.0));
  CHECK(state.pose.yInches == doctest::Approx(24.0));
}

TEST_CASE("pure 24 in forward at heading 90 adds 24 to x") {
  ArcOdometry odometry(imuWithTwoWheels());
  odometry.reset({0.0, 0.0, 90.0});
  odometry.update(straightFrame(0.0, 0.0));
  const RobotState state = odometry.update(straightFrame(0.0, 24.0));
  CHECK(state.pose.xInches == doctest::Approx(24.0));
  CHECK(state.pose.yInches == doctest::Approx(0.0).epsilon(1e-9));
  CHECK(state.pose.headingDegrees == doctest::Approx(90.0));
}

TEST_CASE("pure strafe: 10 in right at 0 is +x, at 90 is -y") {
  ArcOdometry odometry(imuWithTwoWheels());
  odometry.reset({0.0, 0.0, 0.0});
  odometry.update(straightFrame(0.0, 0.0));
  RobotState state = odometry.update(straightFrame(0.0, 0.0, 10.0));
  CHECK(state.pose.xInches == doctest::Approx(10.0));
  CHECK(state.pose.yInches == doctest::Approx(0.0));

  odometry.reset({0.0, 0.0, 90.0});
  odometry.update(straightFrame(0.0, 0.0));
  state = odometry.update(straightFrame(0.0, 0.0, 10.0));
  CHECK(state.pose.xInches == doctest::Approx(0.0).epsilon(1e-9));
  CHECK(state.pose.yInches == doctest::Approx(-10.0));
}

TEST_CASE("known arc: left 10, right 12, wheels at +-6, no IMU") {
  ArcOdometry odometry({.verticalWheelCount = 2, .verticalWheelXInches = {-6.0, 6.0}});
  odometry.reset({0.0, 0.0, 0.0});
  SensorFrame frame;
  frame.verticalWheels[0] = {10.0, true};
  frame.verticalWheels[1] = {12.0, true};
  const RobotState state = odometry.update(frame);

  // Radius 66 in about a center 66 in to the robot's left, turned 1/6 rad counterclockwise.
  CHECK(state.pose.headingDegrees == doctest::Approx(radiansToDegrees(-1.0 / 6.0)));
  CHECK(state.pose.xInches == doctest::Approx(-66.0 * (1.0 - std::cos(1.0 / 6.0))));
  CHECK(state.pose.yInches == doctest::Approx(66.0 * std::sin(1.0 / 6.0)));
  CHECK(state.health == OdometryHealth::Ok);
}

TEST_CASE("the small-angle branch matches the full formula on either side of 1e-6 rad") {
  for (const double turnRadians : {1e-7, 1e-5}) {
    CAPTURE(turnRadians);
    ArcOdometry odometry({.verticalWheelCount = 1, .hasImu = true});
    odometry.reset({0.0, 0.0, 0.0});
    odometry.update(straightFrame(0.0, 0.0));
    const RobotState state = odometry.update(straightFrame(radiansToDegrees(turnRadians), 10.0));
    const double chord = 10.0 * (1.0 - turnRadians * turnRadians / 24.0);
    const double expectedY = chord * std::cos(turnRadians / 2.0);
    const double expectedX = chord * std::sin(turnRadians / 2.0);
    CHECK(state.pose.yInches == doctest::Approx(expectedY).epsilon(1e-12));
    CHECK(state.pose.xInches == doctest::Approx(expectedX).epsilon(1e-12));
  }
}

TEST_CASE("the IMU scale multiplies the IMU rotation") {
  ArcOdometryConfig config = imuWithTwoWheels();
  config.imuScale = 0.9;
  ArcOdometry odometry(config);
  odometry.reset({0.0, 0.0, 0.0});
  odometry.update(straightFrame(0.0, 0.0));
  CHECK(odometry.update(straightFrame(100.0, 0.0)).pose.headingDegrees == doctest::Approx(90.0));
}

TEST_CASE("reset sets the pose and re-baselines the IMU, so there is no jump") {
  ArcOdometry odometry(imuWithTwoWheels());
  odometry.reset({0.0, 0.0, 0.0});
  odometry.update(straightFrame(0.0, 0.0));
  odometry.update(straightFrame(500.0, 0.0));
  odometry.reset({12.0, -3.0, 45.0});
  const RobotState state = odometry.update(straightFrame(500.0, 0.0));
  CHECK(state.pose.xInches == doctest::Approx(12.0));
  CHECK(state.pose.yInches == doctest::Approx(-3.0));
  CHECK(state.pose.headingDegrees == doctest::Approx(45.0));
}

TEST_CASE("lost IMU: heading from drive encoders over W_eff, flagged degraded") {
  ArcOdometry odometry(imuWithTwoWheels());
  odometry.reset({0.0, 0.0, 0.0});
  odometry.update(straightFrame(0.0, 0.0));
  SensorFrame frame = spinFrame(0.0, 10.0);
  frame.imuValid = false;
  const RobotState state = odometry.update(frame);
  CHECK(state.pose.headingDegrees == doctest::Approx(10.0));
  CHECK(state.health == OdometryHealth::Degraded);
}

TEST_CASE("lost horizontal wheel: no sideways motion, degraded") {
  ArcOdometry odometry(imuWithTwoWheels());
  odometry.reset({0.0, 0.0, 0.0});
  odometry.update(straightFrame(0.0, 0.0));
  SensorFrame frame = straightFrame(0.0, 5.0, 3.0);
  frame.horizontalWheel = {0.0, false};
  const RobotState state = odometry.update(frame);
  CHECK(state.pose.xInches == doctest::Approx(0.0));
  CHECK(state.pose.yInches == doctest::Approx(5.0));
  CHECK(state.health == OdometryHealth::Degraded);
}

TEST_CASE("lost one of two vertical wheels: the other one carries on, degraded") {
  ArcOdometry odometry({.verticalWheelCount = 2,
                        .verticalWheelXInches = {-6.0, 6.0},
                        .hasImu = true,
                        .effectiveTrackWidthInches = 12.0});
  odometry.reset({0.0, 0.0, 0.0});
  odometry.update(straightFrame(0.0, 0.0));
  SensorFrame frame = straightFrame(0.0, 4.0);
  frame.verticalWheels[1] = {4.0, true};
  frame.verticalWheels[0] = {0.0, false};
  const RobotState state = odometry.update(frame);
  CHECK(state.pose.yInches == doctest::Approx(4.0));
  CHECK(state.health == OdometryHealth::Degraded);
}

TEST_CASE("lost every vertical wheel: drive encoders take over, degraded") {
  ArcOdometry odometry(imuWithTwoWheels());
  odometry.reset({0.0, 0.0, 0.0});
  odometry.update(straightFrame(0.0, 0.0));
  SensorFrame frame = straightFrame(0.0, 6.0);
  frame.verticalWheels[0] = {0.0, false};
  const RobotState state = odometry.update(frame);
  CHECK(state.pose.yInches == doctest::Approx(6.0));
  CHECK(state.health == OdometryHealth::Degraded);
}

TEST_CASE("no forward source at all is faulted") {
  ArcOdometry odometry(imuWithTwoWheels());
  odometry.reset({0.0, 0.0, 0.0});
  SensorFrame frame = straightFrame(0.0, 6.0);
  frame.verticalWheels[0] = {0.0, false};
  frame.leftDrive = {0.0, false};
  frame.rightDrive = {0.0, false};
  CHECK(odometry.update(frame).health == OdometryHealth::Faulted);
}

TEST_CASE("drive encoders only (no tracking wheels, IMU heading) is a normal setup") {
  ArcOdometry odometry({.hasImu = true, .effectiveTrackWidthInches = 12.0});
  odometry.reset({0.0, 0.0, 0.0});
  odometry.update(straightFrame(0.0, 0.0));
  const RobotState state = odometry.update(straightFrame(0.0, 6.0));
  CHECK(state.pose.yInches == doctest::Approx(6.0));
  CHECK(state.health == OdometryHealth::Ok);
}

TEST_CASE("drive encoders standing in for wheels use offsets -W/2 and +W/2") {
  ArcOdometry odometry({.hasImu = true, .effectiveTrackWidthInches = 12.0});
  odometry.reset({0.0, 0.0, 0.0});
  odometry.update(straightFrame(0.0, 0.0));
  CHECK(maxSpinDrift(odometry) < 1e-9);
}

TEST_CASE("velocity: 0.24 in per 10 ms settles at 24 in/s") {
  ArcOdometry odometry(imuWithTwoWheels());
  odometry.reset({0.0, 0.0, 0.0});
  RobotState state;
  for (int i = 0; i < 60; ++i) {
    state = odometry.update(straightFrame(0.0, 0.24));
  }
  CHECK(state.linearVelocityInchesPerSecond == doctest::Approx(24.0));
  CHECK(state.angularVelocityDegreesPerSecond == doctest::Approx(0.0));
}

TEST_CASE("wheels turning while the IMU says still marks the wheel heading suspect") {
  ArcOdometry odometry(imuWithTwoWheels());
  odometry.reset({0.0, 0.0, 0.0});
  odometry.update(straightFrame(0.0, 0.0));
  RobotState state;
  for (int i = 0; i < 5; ++i) {
    SensorFrame frame = spinFrame(0.0, 1.0);
    state = odometry.update(frame);
  }
  CHECK(state.wheelHeadingSuspect);
  CHECK(state.pose.headingDegrees == doctest::Approx(0.0));
}
