// turn_test.cpp
// Why: proves turn() and turnToPoint() turn in place, the right way (SPEC section 8).
// Owns: one-step tests for algorithms/motion/turn.
#include "cardLib/algorithms/motion/turn.hpp"

#include "doctest.h"

using namespace cardLib;

namespace {

const TurnSettings SETTINGS{.turnGains = {.kP = 2.0}};
constexpr SlewRates NO_SLEW{};

RobotState stateAt(double x, double y, double heading) {
  return {.pose = {x, y, heading}};
}

}  // namespace

TEST_CASE("turn to 90 from 0 spins clockwise in place: left forward, right backward") {
  Turn turn(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  turn.configureHeading(90.0, {});
  turn.start(stateAt(0.0, 0.0, 0.0));
  const MotionStep step = turn.step(stateAt(0.0, 0.0, 0.0), {}, pipeline);
  CHECK(step.command.leftMillivolts > 0);
  CHECK(step.command.rightMillivolts == -step.command.leftMillivolts);
}

TEST_CASE("turn takes the shortest way: 350 from 10 goes counterclockwise") {
  Turn turn(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  turn.configureHeading(350.0, {});
  turn.start(stateAt(0.0, 0.0, 10.0));
  CHECK(turn.step(stateAt(0.0, 0.0, 10.0), {}, pipeline).command.leftMillivolts < 0);
}

TEST_CASE("a forced direction goes the long way") {
  Turn turn(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  turn.configureHeading(350.0, {.direction = TurnDirection::Clockwise});
  turn.start(stateAt(0.0, 0.0, 10.0));
  CHECK(turn.step(stateAt(0.0, 0.0, 10.0), {}, pipeline).command.leftMillivolts > 0);
}

TEST_CASE("after a forced turn arrives, a small overshoot is corrected back, not lapped") {
  Turn turn(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  turn.configureHeading(0.0, {.direction = TurnDirection::Clockwise});
  turn.start(stateAt(0.0, 0.0, 10.0));
  turn.step(stateAt(0.0, 0.0, 10.0), {}, pipeline);
  turn.step(stateAt(0.0, 0.0, 200.0), {.elapsedMs = 10}, pipeline);
  // The slew holds a reversal at 0 for one loop (SPEC section 7), so look at the loop after.
  const MotionStep reversing = turn.step(stateAt(0.0, 0.0, 365.0), {.elapsedMs = 20}, pipeline);
  CHECK(reversing.command.leftMillivolts == 0);
  const MotionStep overshoot = turn.step(stateAt(0.0, 0.0, 365.0), {.elapsedMs = 30}, pipeline);
  CHECK(overshoot.command.leftMillivolts < 0);
}

TEST_CASE("turnToPoint aims at the point, recomputed every step") {
  Turn turn(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  turn.configurePoint({10.0, 0.0}, {});
  turn.start(stateAt(0.0, 0.0, 90.0));
  const MotionStep facing = turn.step(stateAt(0.0, 0.0, 90.0), {}, pipeline);
  CHECK(facing.command.leftMillivolts == 0);
  const MotionStep moved = turn.step(stateAt(0.0, 10.0, 90.0), {.elapsedMs = 10}, pipeline);
  CHECK(moved.command.leftMillivolts > 0);
}

TEST_CASE("turnToPoint backwards faces away from the point") {
  Turn turn(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  turn.configurePoint({10.0, 0.0}, {.forwards = false});
  turn.start(stateAt(0.0, 0.0, -90.0));
  CHECK(turn.step(stateAt(0.0, 0.0, -90.0), {}, pipeline).command.leftMillivolts == 0);
}

TEST_CASE("turnToPoint on top of the point keeps its last target") {
  Turn turn(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  turn.configurePoint({0.0, 0.0}, {});
  turn.start(stateAt(0.0, 0.0, 30.0));
  CHECK(turn.step(stateAt(0.0, 0.0, 30.0), {}, pipeline).command.leftMillivolts == 0);
}

TEST_CASE("progress is degrees turned since the start") {
  Turn turn(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  turn.configureHeading(-90.0, {});
  turn.start(stateAt(0.0, 0.0, 0.0));
  CHECK(turn.step(stateAt(0.0, 0.0, -30.0), {}, pipeline).progress == doctest::Approx(30.0));
}

TEST_CASE("earlyExit ends the turn without braking") {
  Turn turn(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  turn.configureHeading(90.0, {.minSpeed = 30.0, .earlyExit = 5.0});
  turn.start(stateAt(0.0, 0.0, 0.0));
  const MotionStep step = turn.step(stateAt(0.0, 0.0, 87.0), {}, pipeline);
  REQUIRE(step.exit.has_value());
  CHECK(step.exit->result == MotionResult::Settled);
  CHECK_FALSE(step.exit->brake);
}

TEST_CASE("the step reports target and measured heading for the log") {
  Turn turn(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  turn.configureHeading(45.0, {});
  turn.start(stateAt(0.0, 0.0, 0.0));
  const MotionStep step = turn.step(stateAt(0.0, 0.0, 12.0), {}, pipeline);
  CHECK(step.target == doctest::Approx(45.0));
  CHECK(step.measured == doctest::Approx(12.0));
}
