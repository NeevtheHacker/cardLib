// holonomic_motion_test.cpp
// Why: proves the SPEC 20.10 motion laws one step at a time: strafe progress and wheel signs,
//      the holonomic moveToPoint's direction and held heading, the holonomic moveToPose's
//      heading gate, and pivots about a point (SPEC 20.9).
// Owns: tests for algorithms/motion/strafe, holonomic_move_to_point, holonomic_move_to_pose
//       and PivotTurn about a point.
#include <cmath>
#include <numbers>

#include "cardLib/algorithms/motion/holonomic_move_to_point.hpp"
#include "cardLib/algorithms/motion/holonomic_move_to_pose.hpp"
#include "cardLib/algorithms/motion/pivot_turn.hpp"
#include "cardLib/algorithms/motion/strafe.hpp"
#include "doctest.h"

using namespace cardLib;

namespace {

constexpr double CORNER_TOP = std::numbers::pi * 2.75 * 450.0 / 60.0;
constexpr double CENTER_TOP = std::numbers::pi * 2.75 * 600.0 / 60.0;
constexpr DriveKinematics ASTERISK{
    {-5.5, 5.0, 45.0, CORNER_TOP}, {5.5, 5.0, -45.0, CORNER_TOP}, {-5.5, -5.0, -45.0, CORNER_TOP},
    {5.5, -5.0, 45.0, CORNER_TOP}, {-6.0, 0.0, 0.0, CENTER_TOP},  {6.0, 0.0, 0.0, CENTER_TOP},
};
constexpr SlewRates NO_SLEW{};

DrivePipeline asteriskPipeline() {
  return DrivePipeline(ASTERISK, NO_SLEW, NO_SLEW, NO_SLEW);
}

RobotState stateAt(double x, double y, double heading) {
  return {.pose = {x, y, heading}};
}

}  // namespace

TEST_CASE("strafe progress is distance to the right of the start heading") {
  CHECK(strafeProgressInches({0, 0, 0}, {5, 2, 0}) == doctest::Approx(5.0));
  CHECK(strafeProgressInches({0, 0, 90}, {0, -5, 90}) == doctest::Approx(5.0));
  CHECK(strafeProgressInches({1, 1, 180}, {-3, 1, 180}) == doctest::Approx(4.0));
}

TEST_CASE("strafe right drives the corners in the X pattern and leaves the centers") {
  DrivePipeline pipeline = asteriskPipeline();
  Strafe strafe({.strafeGains = {.kP = 10.0}, .headingGains = {.kP = 2.0}});
  strafe.configure(24.0, {});
  strafe.start(stateAt(0, 0, 0));
  const MotionStep step = strafe.step(stateAt(0, 0, 0), {}, pipeline);
  CHECK(step.command.wheels[0].millivolts > 0);
  CHECK(step.command.wheels[1].millivolts < 0);
  CHECK(step.command.wheels[2].millivolts < 0);
  CHECK(step.command.wheels[3].millivolts > 0);
  CHECK(step.command.wheels[4].millivolts == 0);
  CHECK(step.target == 24.0);
}

TEST_CASE("strafe holds the start heading: turned counterclockwise, it corrects clockwise") {
  DrivePipeline pipeline = asteriskPipeline();
  Strafe strafe({.strafeGains = {.kP = 0.0}, .headingGains = {.kP = 2.0}});
  strafe.configure(24.0, {});
  strafe.start(stateAt(0, 0, 0));
  const MotionStep step = strafe.step(stateAt(0, 0, -10.0), {}, pipeline);
  CHECK(step.command.wheels[4].millivolts > 0);  // left center forward
  CHECK(step.command.wheels[5].millivolts < 0);  // right center back
}

TEST_CASE("translationToward points the physical path along the field direction") {
  const TranslationChannels channels = translationToward(
      {0, 0, 0}, {1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0)}, 100.0, ASTERISK, 127.0);
  // Physical speeds: forward = lin / 127 * maxForward, right = strafe / 127 * maxStrafe.
  const double forward = channels.forward.speed / 127.0 * ASTERISK.maxForwardInchesPerSecond();
  const double right = channels.strafe.speed / 127.0 * ASTERISK.maxStrafeInchesPerSecond();
  CHECK(forward == doctest::Approx(right));
  CHECK(forward > 0.0);
}

TEST_CASE("translationToward turns field directions into robot directions") {
  // Facing +x (90 degrees), a target at +x is straight ahead.
  const TranslationChannels channels =
      translationToward({0, 0, 90}, {1.0, 0.0}, 50.0, ASTERISK, 127.0);
  CHECK(channels.forward.speed == doctest::Approx(50.0));
  CHECK(channels.strafe.speed == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("holonomic moveToPoint holds the start heading unless told otherwise") {
  HolonomicMoveToPoint move({.translationGains = {.kP = 5.0}, .headingGains = {.kP = 2.0}});
  move.configure({10.0, 0.0}, {});
  move.start(stateAt(0, 0, 30.0));
  CHECK(move.headingErrorDegrees({0, 0, 30.0}) == doctest::Approx(0.0));
  move.configure({10.0, 0.0}, {.headingDegrees = 90.0});
  move.start(stateAt(0, 0, 30.0));
  CHECK(move.headingErrorDegrees({0, 0, 30.0}) == doctest::Approx(60.0));
}

TEST_CASE("holonomic moveToPoint to the right of the robot strafes, it doesn't turn") {
  DrivePipeline pipeline = asteriskPipeline();
  HolonomicMoveToPoint move({.translationGains = {.kP = 5.0}, .headingGains = {.kP = 2.0}});
  move.configure({10.0, 0.0}, {});
  move.start(stateAt(0, 0, 0));
  const MotionStep step = move.step(stateAt(0, 0, 0), {}, pipeline);
  CHECK(step.command.wheels[0].millivolts > 0);
  CHECK(step.command.wheels[1].millivolts < 0);
  CHECK(step.command.wheels[4].millivolts == 0);
  CHECK(step.measured == doctest::Approx(10.0));
}

TEST_CASE("holonomic moveToPose won't settle at the point until the heading has settled") {
  DrivePipeline pipeline = asteriskPipeline();
  HolonomicMoveToPose move(
      {.drive = {.translationGains = {.kP = 5.0}, .headingGains = {.kP = 2.0}}});
  move.configure({0.0, 0.0, 90.0}, {});
  move.start(stateAt(0, 0, 0));
  for (std::uint32_t t = 0; t <= 1000; t += 10) {
    CHECK_FALSE(move.step(stateAt(0, 0, 0), {.elapsedMs = t}, pipeline).exit.has_value());
  }
  std::optional<ExitDecision> exit;
  for (std::uint32_t t = 1010; t <= 1500 && !exit.has_value(); t += 10) {
    exit = move.step(stateAt(0, 0, 90.0), {.elapsedMs = t}, pipeline).exit;
  }
  REQUIRE(exit.has_value());
  CHECK(exit->result == MotionResult::Settled);
}

TEST_CASE("a pivot about a point holds the wheel at that point") {
  DrivePipeline pipeline = asteriskPipeline();
  PivotTurn pivot({.pivotGains = {.kP = 3.0}});
  pivot.configure(90.0, LocalOffset{.rightInches = 5.5, .forwardInches = -5.0}, {});
  pivot.start(stateAt(0, 0, 0));
  const MotionStep step = pivot.step(stateAt(0, 0, 0), {}, pipeline);
  CHECK(step.command.wheels[3].mode == SideMode::Held);
  CHECK(step.command.wheels[0].mode == SideMode::Driven);
}

TEST_CASE("a side pivot still works after an about-point pivot") {
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  PivotTurn pivot({.pivotGains = {.kP = 3.0}});
  pivot.configure(90.0, LocalOffset{.rightInches = 0.5}, {});
  pivot.configure(90.0, Side::Left, {});
  pivot.start(stateAt(0, 0, 0));
  const MotionStep step = pivot.step(stateAt(0, 0, 0), {}, pipeline);
  CHECK(step.command.leftMode == SideMode::Held);
  CHECK(step.command.rightMillivolts < 0);
}
