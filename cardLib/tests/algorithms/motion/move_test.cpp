// move_test.cpp
// Why: proves move() measures progress along its start heading, holds that heading and settles
//      direction before the loop (SPEC section 8).
// Owns: one-step tests for algorithms/motion/move.
#include "cardLib/algorithms/motion/move.hpp"

#include <cstdint>
#include <optional>

#include "doctest.h"

using namespace cardLib;

namespace {

const MoveSettings SETTINGS{.linearGains = {.kP = 10.0}, .headingGains = {.kP = 2.0}};
constexpr SlewRates NO_SLEW{};

RobotState stateAt(double x, double y, double heading) {
  return {.pose = {x, y, heading}};
}

}  // namespace

TEST_CASE("progress is measured along the start heading; sideways drift doesn't count") {
  CHECK(moveProgressInches({0.0, 0.0, 0.0}, {3.0, 10.0, 0.0}) == doctest::Approx(10.0));
  CHECK(moveProgressInches({0.0, 0.0, 90.0}, {10.0, 3.0, 90.0}) == doctest::Approx(10.0));
  CHECK(moveProgressInches({5.0, 5.0, 180.0}, {5.0, -5.0, 180.0}) == doctest::Approx(10.0));
  CHECK(moveProgressInches({0.0, 0.0, 0.0}, {0.0, -4.0, 0.0}) == doctest::Approx(-4.0));
}

TEST_CASE("direction is settled before the loop: forwards=false or a negative distance") {
  CHECK(signedMoveTarget(24.0, true) == doctest::Approx(24.0));
  CHECK(signedMoveTarget(-24.0, true) == doctest::Approx(-24.0));
  CHECK(signedMoveTarget(24.0, false) == doctest::Approx(-24.0));
  CHECK(signedMoveTarget(-24.0, false) == doctest::Approx(-24.0));
}

TEST_CASE("move forward drives both sides forward at first") {
  Move move(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  move.configure(24.0, {});
  move.start(stateAt(0.0, 0.0, 0.0));
  const MotionStep step = move.step(stateAt(0.0, 0.0, 0.0), {}, pipeline);
  CHECK_FALSE(step.exit.has_value());
  CHECK(step.command.leftMillivolts == 12000);
  CHECK(step.command.rightMillivolts == 12000);
}

TEST_CASE("move backwards drives both sides backwards") {
  Move move(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  move.configure(24.0, {.forwards = false});
  move.start(stateAt(0.0, 0.0, 0.0));
  const MotionStep step = move.step(stateAt(0.0, 0.0, 0.0), {}, pipeline);
  CHECK(step.command.leftMillivolts == -12000);
  CHECK(step.command.rightMillivolts == -12000);
}

TEST_CASE("heading hold: drifted clockwise, the robot steers counterclockwise") {
  Move move(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  move.configure(2.0, {});
  move.start(stateAt(0.0, 0.0, 0.0));
  const MotionStep step = move.step(stateAt(0.0, 0.0, 5.0), {}, pipeline);
  CHECK(step.command.rightMillivolts > step.command.leftMillivolts);
}

TEST_CASE("progress for waitUntil is inches along the start heading") {
  Move move(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  move.configure(-24.0, {});
  move.start(stateAt(0.0, 0.0, 0.0));
  CHECK(move.step(stateAt(1.0, -6.0, 0.0), {}, pipeline).progress == doctest::Approx(6.0));
}

TEST_CASE("degraded odometry refuses the move unless allowed") {
  Move move(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  RobotState degraded = stateAt(0.0, 0.0, 0.0);
  degraded.health = OdometryHealth::Degraded;

  move.configure(24.0, {});
  move.start(degraded);
  auto step = move.step(degraded, {}, pipeline);
  REQUIRE(step.exit.has_value());
  CHECK(step.exit->result == MotionResult::SensorFault);

  move.configure(24.0, {.allowDegraded = true});
  move.start(degraded);
  step = move.step(degraded, {}, pipeline);
  CHECK_FALSE(step.exit.has_value());
}

TEST_CASE("the move settles on its own small window") {
  Move move(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  move.configure(24.0, {});
  move.start(stateAt(0.0, 0.0, 0.0));
  std::optional<ExitDecision> exit;
  for (std::uint32_t timeMs = 0; timeMs <= 200 && !exit; timeMs += 10) {
    exit = move.step(stateAt(0.0, 23.5, 0.0), {.elapsedMs = timeMs}, pipeline).exit;
  }
  REQUIRE(exit.has_value());
  CHECK(exit->result == MotionResult::Settled);
}

TEST_CASE("the step reports target and measured progress for the log") {
  Move move(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  move.configure(24.0, {.forwards = false});
  move.start(stateAt(0.0, 0.0, 0.0));
  const MotionStep step = move.step(stateAt(0.0, -5.0, 0.0), {}, pipeline);
  CHECK(step.target == doctest::Approx(-24.0));
  CHECK(step.measured == doctest::Approx(-5.0));
}
