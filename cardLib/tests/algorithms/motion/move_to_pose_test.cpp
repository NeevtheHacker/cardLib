// move_to_pose_test.cpp
// Why: proves moveToPose() hands off from driving to turning only when close and slow, and
//      reports one result over one shared timeout (SPEC section 8).
// Owns: one-step tests for algorithms/motion/move_to_pose.
#include "cardLib/algorithms/motion/move_to_pose.hpp"

#include <cstdint>
#include <optional>

#include "doctest.h"

using namespace cardLib;

namespace {

const MoveToPoseSettings SETTINGS{
    .drive = {.linearGains = {.kP = 5.0}, .headingGains = {.kP = 1.0}},
    .turn = {.turnGains = {.kP = 2.0}},
};
constexpr SlewRates NO_SLEW{};

RobotState stateAt(double x, double y, double heading, double speed = 0.0) {
  return {.pose = {x, y, heading}, .linearVelocityInchesPerSecond = speed};
}

}  // namespace

TEST_CASE("far from the target it drives") {
  MoveToPose motion(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, 24.0, 90.0}, {});
  motion.start(stateAt(0.0, 0.0, 0.0));
  motion.step(stateAt(0.0, 0.0, 0.0), {}, pipeline);
  CHECK(motion.phase() == MoveToPose::Phase::Drive);
  CHECK(pipeline.linearSpeed() > 0.0);
}

TEST_CASE("close but still fast: no hand-off yet") {
  MoveToPose motion(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, 24.0, 90.0}, {});
  motion.start(stateAt(0.0, 0.0, 0.0));
  motion.step(stateAt(0.0, 23.5, 0.0, 10.0), {}, pipeline);
  CHECK(motion.phase() == MoveToPose::Phase::Drive);
}

TEST_CASE("slow but not close: no hand-off, and the drive phase never settles on its own") {
  MoveToPose motion(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, 24.0, 90.0}, {});
  motion.start(stateAt(0.0, 0.0, 0.0));
  for (std::uint32_t timeMs = 0; timeMs < 290; timeMs += 10) {
    const MotionStep step = motion.step(stateAt(0.0, 21.5, 0.0, 1.5), {.elapsedMs = timeMs},
                                        pipeline);
    CHECK_FALSE(step.exit.has_value());
  }
  CHECK(motion.phase() == MoveToPose::Phase::Drive);
}

TEST_CASE("close and slow: hands off to an in-place turn") {
  MoveToPose motion(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, 24.0, 90.0}, {});
  motion.start(stateAt(0.0, 0.0, 0.0));
  const MotionStep step = motion.step(stateAt(0.0, 23.5, 0.0, 1.0), {}, pipeline);
  CHECK(motion.phase() == MoveToPose::Phase::Turn);
  CHECK(step.command.leftMillivolts > 0);
  CHECK(step.command.rightMillivolts == -step.command.leftMillivolts);
}

TEST_CASE("already facing the final heading: the turn is skipped") {
  MoveToPose motion(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, 24.0, 0.5}, {});
  motion.start(stateAt(0.0, 0.0, 0.0));
  const MotionStep step = motion.step(stateAt(0.0, 23.5, 0.0, 1.0), {}, pipeline);
  REQUIRE(step.exit.has_value());
  CHECK(step.exit->result == MotionResult::Settled);
}

TEST_CASE("a drive-phase failure is the motion's result") {
  MoveToPose motion(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, 24.0, 90.0}, {});
  motion.start(stateAt(0.0, 0.0, 0.0));
  const MotionStep step = motion.step(stateAt(0.0, 0.0, 0.0), {.cancelled = true}, pipeline);
  REQUIRE(step.exit.has_value());
  CHECK(step.exit->result == MotionResult::Cancelled);
}

TEST_CASE("one timeout covers both phases") {
  MoveToPose motion(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, 24.0, 90.0}, {.timeoutMs = 1000});
  motion.start(stateAt(0.0, 0.0, 0.0));
  motion.step(stateAt(0.0, 23.5, 0.0, 1.0), {.elapsedMs = 900}, pipeline);
  REQUIRE(motion.phase() == MoveToPose::Phase::Turn);
  const MotionStep step = motion.step(stateAt(0.0, 23.5, 30.0, 0.0), {.elapsedMs = 1010}, pipeline);
  REQUIRE(step.exit.has_value());
  CHECK(step.exit->result == MotionResult::TimedOut);
}

TEST_CASE("the turn phase's result is the motion's result") {
  MoveToPose motion(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, 24.0, 90.0}, {});
  motion.start(stateAt(0.0, 0.0, 0.0));
  motion.step(stateAt(0.0, 23.5, 0.0, 1.0), {}, pipeline);
  std::optional<ExitDecision> exit;
  for (std::uint32_t timeMs = 10; timeMs <= 300 && !exit; timeMs += 10) {
    exit = motion.step(stateAt(0.0, 23.5, 89.5), {.elapsedMs = timeMs}, pipeline).exit;
  }
  REQUIRE(exit.has_value());
  CHECK(exit->result == MotionResult::Settled);
}
