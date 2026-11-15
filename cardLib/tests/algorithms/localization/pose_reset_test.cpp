// pose_reset_test.cpp
// Why: proves the SPEC 21 resets: a distance sensor snaps one coordinate to a wall, the GPS
//      snaps x, y and optionally heading, and each one refuses (pose untouched) for every
//      reason the spec lists.
// Owns: tests for algorithms/localization/pose_reset.
#include "cardLib/algorithms/localization/pose_reset.hpp"

#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"
#include "doctest.h"

using namespace cardLib;

namespace {

constexpr FieldWalls WALLS{};  // a 144 in field with its origin at the center

RobotState stoppedAt(double x, double y, double heading) {
  return {.pose = {x, y, heading}};
}

DistanceReading readingOf(double inches) {
  return {.inches = inches, .confidence = 1.0, .valid = true};
}

// A sensor on the robot's left side, pointing left.
constexpr DistanceMount LEFT_SENSOR{.xInches = -6.0, .yInches = 0.0, .headingDegrees = -90.0};
// A sensor on the robot's back, pointing back.
constexpr DistanceMount BACK_SENSOR{.xInches = 0.0, .yInches = -6.0, .headingDegrees = 180.0};

}  // namespace

TEST_CASE("left wall: the left sensor sets x and nothing else") {
  const PoseResetDecision decision =
      wallReset(stoppedAt(-57.0, 10.0, 0.0), LEFT_SENSOR, readingOf(10.0), Wall::Left, WALLS, {});
  CHECK(decision.result == ResetResult::Applied);
  CHECK(decision.pose.xInches == doctest::Approx(-72.0 + 10.0 + 6.0));
  CHECK(decision.pose.yInches == 10.0);
  CHECK(decision.pose.headingDegrees == 0.0);
  CHECK(decision.changeInches == doctest::Approx(1.0));
}

TEST_CASE("facing +x, the back sensor faces the left wall; its offset turns with the robot") {
  const PoseResetDecision decision =
      wallReset(stoppedAt(-55.0, 0.0, 90.0), BACK_SENSOR, readingOf(12.0), Wall::Left, WALLS, {});
  REQUIRE(decision.result == ResetResult::Applied);
  CHECK(decision.pose.xInches == doctest::Approx(-72.0 + 12.0 + 6.0));
  CHECK(decision.pose.headingDegrees == 90.0);
}

TEST_CASE("right, back and front walls") {
  const DistanceMount right{.xInches = 6.0, .headingDegrees = 90.0};
  CHECK(wallReset(stoppedAt(55.0, 0, 0), right, readingOf(10.0), Wall::Right, WALLS, {})
            .pose.xInches == doctest::Approx(72.0 - 10.0 - 6.0));
  CHECK(wallReset(stoppedAt(0, -55.0, 0), BACK_SENSOR, readingOf(10.0), Wall::Back, WALLS, {})
            .pose.yInches == doctest::Approx(-72.0 + 10.0 + 6.0));
  const DistanceMount front{.yInches = 6.0, .headingDegrees = 0.0};
  const PoseResetDecision decision =
      wallReset(stoppedAt(3.0, 55.0, 0), front, readingOf(10.0), Wall::Front, WALLS, {});
  CHECK(decision.pose.yInches == doctest::Approx(72.0 - 10.0 - 6.0));
  CHECK(decision.pose.xInches == 3.0);
}

TEST_CASE("a few degrees off square: the reading is projected onto the wall's axis") {
  const RobotState state = stoppedAt(-57.0, 0.0, 10.0);
  const PoseResetDecision decision =
      wallReset(state, LEFT_SENSOR, readingOf(10.0), Wall::Left, WALLS, {});
  REQUIRE(decision.result == ResetResult::Applied);
  // Sensor x on the field, from the robot center turned 10 degrees.
  const double sensorOffsetX = -6.0 * std::cos(degreesToRadians(10.0));
  CHECK(decision.pose.xInches ==
        doctest::Approx(-72.0 + 10.0 * std::cos(degreesToRadians(10.0)) - sensorOffsetX));
}

TEST_CASE("refusals leave the pose untouched and say why") {
  const RobotState state = stoppedAt(-57.0, 10.0, 0.0);
  const auto check = [&](const PoseResetDecision& decision, ResetResult expected) {
    CHECK(decision.result == expected);
    CHECK(decision.pose.xInches == state.pose.xInches);
    CHECK(decision.pose.yInches == state.pose.yInches);
    CHECK(decision.pose.headingDegrees == state.pose.headingDegrees);
  };
  check(wallReset(state, LEFT_SENSOR, {}, Wall::Left, WALLS, {}), ResetResult::NoReading);
  check(wallReset(state, LEFT_SENSOR, readingOf(90.0), Wall::Left, WALLS, {}),
        ResetResult::OutOfRange);
  check(wallReset(state, LEFT_SENSOR, readingOf(0.5), Wall::Left, WALLS, {}),
        ResetResult::OutOfRange);
  const RobotState turned = stoppedAt(-57.0, 10.0, 20.0);
  const PoseResetDecision offSquare =
      wallReset(turned, LEFT_SENSOR, readingOf(10.0), Wall::Left, WALLS, {});
  CHECK(offSquare.result == ResetResult::NotSquare);
  CHECK(offSquare.pose.xInches == turned.pose.xInches);
  CHECK(offSquare.pose.headingDegrees == turned.pose.headingDegrees);
  check(wallReset(state, LEFT_SENSOR, readingOf(10.0), Wall::Right, WALLS, {}),
        ResetResult::NotSquare);
  check(wallReset(state, LEFT_SENSOR, readingOf(16.0), Wall::Left, WALLS, {}),
        ResetResult::ChangeTooLarge);
  RobotState moving = state;
  moving.linearVelocityInchesPerSecond = 5.0;
  check(wallReset(moving, LEFT_SENSOR, readingOf(10.0), Wall::Left, WALLS, {}),
        ResetResult::NotStopped);
  RobotState turning = state;
  turning.angularVelocityDegreesPerSecond = 10.0;
  check(wallReset(turning, LEFT_SENSOR, readingOf(10.0), Wall::Left, WALLS, {}),
        ResetResult::NotStopped);
}

TEST_CASE("the change limit and square limit are options") {
  const RobotState state = stoppedAt(-57.0, 10.0, 0.0);
  CHECK(wallReset(state, LEFT_SENSOR, readingOf(16.0), Wall::Left, WALLS, {.maxChangeInches = 8.0})
            .result == ResetResult::Applied);
  CHECK(wallReset(stoppedAt(-57.0, 10.0, 20.0), LEFT_SENSOR, readingOf(10.0), Wall::Left, WALLS,
                  {.maxAngleFromSquareDegrees = 25.0})
            .result == ResetResult::Applied);
}

TEST_CASE("GPS: a sensor at the center, mounted forward, reads the robot pose directly") {
  const Pose pose = robotPoseFromGps({10.0, -20.0, 45.0}, {});
  CHECK(pose.xInches == doctest::Approx(10.0));
  CHECK(pose.yInches == doctest::Approx(-20.0));
  CHECK(pose.headingDegrees == doctest::Approx(45.0));
}

TEST_CASE("GPS: mounted at the back facing backward, as the docs suggest") {
  const GpsMount rear{.yInches = -6.0, .headingDegrees = 180.0};
  // Robot at (0, 30) facing +y: the sensor sits at (0, 24) and faces 180.
  const Pose pose = robotPoseFromGps({0.0, 24.0, 180.0}, rear);
  CHECK(pose.xInches == doctest::Approx(0.0).epsilon(1e-9));
  CHECK(pose.yInches == doctest::Approx(30.0));
  CHECK(wrapDegrees(pose.headingDegrees) == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("GPS: the field frame can be turned and its origin shifted to the team's") {
  const GpsMount mount{
      .fieldXOffsetInches = 72.0, .fieldYOffsetInches = 72.0, .fieldRotationDegrees = 90.0};
  // GPS +y is the team's +x after a 90 degree turn.
  const Pose pose = robotPoseFromGps({0.0, 10.0, 0.0}, mount);
  CHECK(pose.xInches == doctest::Approx(82.0));
  CHECK(pose.yInches == doctest::Approx(72.0));
  CHECK(pose.headingDegrees == doctest::Approx(90.0));
}

TEST_CASE("GPS reset: x and y always, heading only when asked, kept continuous") {
  const RobotState state = stoppedAt(1.0, 2.0, 725.0);
  const GpsReading reading{.robotPose = {3.0, 4.0, 10.0}, .errorInches = 0.5, .valid = true};
  const PoseResetDecision position = gpsReset(state, reading, {});
  CHECK(position.result == ResetResult::Applied);
  CHECK(position.pose.xInches == 3.0);
  CHECK(position.pose.yInches == 4.0);
  CHECK(position.pose.headingDegrees == 725.0);
  CHECK(position.changeInches == doctest::Approx(std::hypot(2.0, 2.0)));
  const PoseResetDecision withHeading = gpsReset(state, reading, {.resetHeading = true});
  CHECK(withHeading.pose.headingDegrees == doctest::Approx(730.0));
}

TEST_CASE("GPS reset refusals") {
  const RobotState state = stoppedAt(1.0, 2.0, 0.0);
  const GpsReading good{.robotPose = {3.0, 4.0, 0.0}, .errorInches = 0.5, .valid = true};
  CHECK(gpsReset(state, {}, {}).result == ResetResult::NoReading);
  GpsReading noisy = good;
  noisy.errorInches = 3.0;
  CHECK(gpsReset(state, noisy, {}).result == ResetResult::ErrorTooHigh);
  GpsReading weak = good;
  weak.qualityPercent = 90.0;
  CHECK(gpsReset(state, weak, {}).result == ResetResult::LowQuality);
  weak.qualityPercent = 99.0;
  CHECK(gpsReset(state, weak, {}).result == ResetResult::Applied);
  RobotState moving = state;
  moving.sidewaysVelocityInchesPerSecond = 4.0;
  CHECK(gpsReset(moving, good, {}).result == ResetResult::NotStopped);
  CHECK(gpsReset(moving, good, {}).pose.xInches == 1.0);
}
