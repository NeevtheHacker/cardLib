// pivot_turn_test.cpp
// Why: proves pivotTurn() drives only the free side, with the SPEC section 8 signs.
// Owns: one-step tests for algorithms/motion/pivot_turn.
#include "cardLib/algorithms/motion/pivot_turn.hpp"

#include <cstdint>
#include <optional>

#include "doctest.h"

using namespace cardLib;

namespace {

const PivotTurnSettings SETTINGS{.pivotGains = {.kP = 2.0}};
constexpr SlewRates NO_SLEW{};

RobotState facing(double heading) {
  return {.pose = {0.0, 0.0, heading}};
}

}  // namespace

TEST_CASE("left side locked, clockwise target: left held, right drives backwards") {
  PivotTurn pivot(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  pivot.configure(45.0, Side::Left, {});
  pivot.start(facing(0.0));
  const MotionStep step = pivot.step(facing(0.0), {}, pipeline);
  CHECK(step.command.leftMillivolts == 0);
  CHECK(step.command.leftMode == SideMode::Held);
  CHECK(step.command.rightMillivolts < 0);
}

TEST_CASE("right side locked, clockwise target: left drives forwards, right held") {
  PivotTurn pivot(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  pivot.configure(45.0, Side::Right, {});
  pivot.start(facing(0.0));
  const MotionStep step = pivot.step(facing(0.0), {}, pipeline);
  CHECK(step.command.leftMillivolts > 0);
  CHECK(step.command.rightMillivolts == 0);
  CHECK(step.command.rightMode == SideMode::Held);
}

TEST_CASE("counterclockwise target with the right side locked drives the left side backwards") {
  PivotTurn pivot(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  pivot.configure(-45.0, Side::Right, {});
  pivot.start(facing(0.0));
  CHECK(pivot.step(facing(0.0), {}, pipeline).command.leftMillivolts < 0);
}

TEST_CASE("pivot settles on its own windows (1.5 degrees)") {
  PivotTurn pivot(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  pivot.configure(45.0, Side::Left, {});
  pivot.start(facing(0.0));
  std::optional<ExitDecision> exit;
  for (std::uint32_t timeMs = 0; timeMs <= 200 && !exit; timeMs += 10) {
    exit = pivot.step(facing(44.0), {.elapsedMs = timeMs}, pipeline).exit;
  }
  REQUIRE(exit.has_value());
  CHECK(exit->result == MotionResult::Settled);
}
