// motion_sim_test.cpp
// Why: the SPEC section 13 simulator tests: whole motions, through the real pipeline and
//      odometry, against a lagging drive model.
// Owns: the sim tests. Gains here suit the simulator only; real gains come from tuning.
#include <algorithm>
#include <cmath>

#include "cardLib/algorithms/localization/plausibility_check.hpp"
#include "cardLib/algorithms/localization/wheel_distance.hpp"
#include "cardLib/algorithms/motion/move.hpp"
#include "cardLib/algorithms/motion/move_to_point.hpp"
#include "cardLib/algorithms/motion/move_to_pose.hpp"
#include "cardLib/algorithms/motion/pivot_turn.hpp"
#include "cardLib/algorithms/motion/turn.hpp"
#include "doctest.h"
#include "sim_robot.hpp"

using namespace cardLib;
using namespace cardLib::sim;

namespace {

constexpr SlewRates LINEAR_SLEW{.accelPerSecond = 500.0, .decelPerSecond = 1500.0};
constexpr SlewRates ANGULAR_SLEW{.accelPerSecond = 1500.0, .decelPerSecond = 3000.0};
constexpr PidGains LINEAR_GAINS{.kP = 8.0, .kD = 0.6};
constexpr PidGains HEADING_GAINS{.kP = 2.0};
constexpr PidGains TURN_GAINS{.kP = 2.0, .kD = 0.2};
constexpr std::uint32_t SETTLE_IDLE_MS = 500;

SimRobot makeRobot(const Pose& start = {}) {
  return SimRobot({}, LINEAR_SLEW, ANGULAR_SLEW, start);
}

double distanceTo(const Pose& pose, double x, double y) {
  return std::hypot(pose.xInches - x, pose.yInches - y);
}

}  // namespace

TEST_CASE("sim: move(24) settles within 1 in") {
  SimRobot robot = makeRobot();
  Move move({.linearGains = LINEAR_GAINS, .headingGains = HEADING_GAINS});
  move.configure(24.0, {});
  const SimRun run = robot.run(move);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(run.result == MotionResult::Settled);
  CHECK(std::abs(robot.drive().truePose().yInches - 24.0) < 1.0);
  CHECK(std::abs(robot.drive().truePose().xInches) < 0.1);
}

TEST_CASE("sim: move(-24) backs up 24 in") {
  SimRobot robot = makeRobot();
  Move move({.linearGains = LINEAR_GAINS, .headingGains = HEADING_GAINS});
  move.configure(24.0, {.forwards = false});
  CHECK(robot.run(move).result == MotionResult::Settled);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(std::abs(robot.drive().truePose().yInches + 24.0) < 1.0);
}

TEST_CASE("sim: turn(90) overshoots by less than 2 degrees") {
  SimRobot robot = makeRobot();
  Turn turn({.turnGains = TURN_GAINS});
  turn.configureHeading(90.0, {});
  const SimRun run = robot.run(turn);
  double maxHeading = 0.0;
  for (const TraceSample& sample : run.trace) {
    maxHeading = std::max(maxHeading, sample.truePose.headingDegrees);
  }
  robot.idle(SETTLE_IDLE_MS);
  CHECK(run.result == MotionResult::Settled);
  CHECK(maxHeading - 90.0 < 2.0);
  CHECK(std::abs(robot.drive().truePose().headingDegrees - 90.0) < 1.0);
  CHECK(distanceTo(robot.drive().truePose(), 0.0, 0.0) < 0.01);
}

TEST_CASE("sim: moveToPoint ends within 1 in") {
  SimRobot robot = makeRobot();
  MoveToPoint motion({.linearGains = LINEAR_GAINS, .headingGains = HEADING_GAINS});
  motion.configure({12.0, 24.0}, {});
  const SimRun run = robot.run(motion);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(run.result == MotionResult::Settled);
  CHECK(distanceTo(robot.drive().truePose(), 12.0, 24.0) < 1.0);
}

TEST_CASE("sim: moveToPoint to a point behind backs toward it") {
  SimRobot robot = makeRobot();
  MoveToPoint motion({.linearGains = LINEAR_GAINS, .headingGains = HEADING_GAINS});
  motion.configure({0.0, -20.0}, {.forwards = false});
  CHECK(robot.run(motion).result == MotionResult::Settled);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(distanceTo(robot.drive().truePose(), 0.0, -20.0) < 1.0);
  CHECK(std::abs(robot.drive().truePose().headingDegrees) < 5.0);
}

TEST_CASE("sim: a chained pair of moves never stops in between") {
  SimRobot robot = makeRobot();
  Move first({.linearGains = LINEAR_GAINS, .headingGains = HEADING_GAINS});
  Move second({.linearGains = LINEAR_GAINS, .headingGains = HEADING_GAINS});
  first.configure(24.0, {.minSpeed = 50.0, .earlyExit = 2.0});
  second.configure(24.0, {});

  const SimRun firstRun = robot.run(first);
  REQUIRE(firstRun.result == MotionResult::Settled);
  CHECK_FALSE(firstRun.brake);
  const double secondStartY = robot.state().pose.yInches;
  const SimRun secondRun = robot.run(second);
  REQUIRE(secondRun.result == MotionResult::Settled);

  double slowestAcrossHandoff = robot.drive().linearSpeedInchesPerSecond() + 1e9;
  for (const TraceSample& sample : secondRun.trace) {
    if (sample.elapsedMs > 200) {
      break;
    }
    slowestAcrossHandoff = std::min(slowestAcrossHandoff, sample.linearSpeedInchesPerSecond);
  }
  CHECK(firstRun.trace.back().linearSpeedInchesPerSecond > 20.0);
  CHECK(slowestAcrossHandoff > 20.0);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(std::abs(robot.drive().truePose().yInches - (secondStartY + 24.0)) < 1.0);
}

TEST_CASE("sim: pivot turns go clockwise about the locked side") {
  for (const Side locked : {Side::Left, Side::Right}) {
    CAPTURE(static_cast<int>(locked));
    SimRobot robot = makeRobot();
    PivotTurn pivot({.pivotGains = TURN_GAINS});
    pivot.configure(90.0, locked, {});
    const SimRun run = robot.run(pivot);
    robot.idle(SETTLE_IDLE_MS);
    CHECK(run.result == MotionResult::Settled);
    CHECK(std::abs(robot.drive().truePose().headingDegrees - 90.0) < 2.0);
    // The center swings a quarter circle about the locked wheel, 6 in to that side: about
    // (-6, 0) it ends at (-6, -6); about (6, 0) it ends at (6, 6).
    const double expectedX = locked == Side::Left ? -6.0 : 6.0;
    const double expectedY = locked == Side::Left ? -6.0 : 6.0;
    CHECK(robot.drive().truePose().xInches == doctest::Approx(expectedX).epsilon(0.05));
    CHECK(robot.drive().truePose().yInches == doctest::Approx(expectedY).epsilon(0.05));
  }
}

TEST_CASE("sim: moveToPose ends within 1 in and 1 degree") {
  SimRobot robot = makeRobot();
  MoveToPose motion({.drive = {.linearGains = LINEAR_GAINS, .headingGains = HEADING_GAINS},
                     .turn = {.turnGains = TURN_GAINS}});
  motion.configure({12.0, 24.0, 90.0}, {});
  const SimRun run = robot.run(motion);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(run.result == MotionResult::Settled);
  CHECK(motion.phase() == MoveToPose::Phase::Turn);
  CHECK(distanceTo(robot.drive().truePose(), 12.0, 24.0) < 1.0);
  CHECK(std::abs(robot.drive().truePose().headingDegrees - 90.0) < 1.0);
}

TEST_CASE("sim: an injected encoder jump trips the plausibility check and adds no motion") {
  SimRobot robot = makeRobot();
  const double maxSpeed = maxWheelSpeedInchesPerSecond(3.25, 450.0);
  PlausibilityCheck check(maxSpeed);
  robot.drive().apply({.leftMillivolts = 6000, .rightMillivolts = 6000});
  double counted = 0.0;
  for (int loop = 0; loop < 100; ++loop) {
    robot.drive().advance(LOOP_PERIOD_SECONDS);
    if (loop == 50) {
      robot.drive().injectLeftEncoderJump(10.0);
    }
    counted += check.update(robot.drive().leftReportedInches(), LOOP_PERIOD_SECONDS);
    if (loop == 50) {
      CHECK(check.flagged());
    }
  }
  const double travelled = robot.drive().leftReportedInches() - 10.0;
  CHECK(counted <= travelled);
  CHECK(counted > travelled - 1.0);
  CHECK_FALSE(check.faulted());
}

TEST_CASE("sim: one rejected encoder reading in the middle of a move does not end it") {
  SimRobot robot = makeRobot();
  Move move({.linearGains = LINEAR_GAINS, .headingGains = HEADING_GAINS});
  move.configure(24.0, {});
  robot.failLeftEncoderReads(300, 1);
  const SimRun run = robot.run(move);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(run.result == MotionResult::Settled);
  for (const TraceSample& sample : run.trace) {
    CHECK(sample.health == OdometryHealth::Ok);
  }
  CHECK(std::abs(robot.drive().truePose().yInches - 24.0) < 1.0);
}

TEST_CASE("sim: two rejected readings in a row are still not a fault") {
  SimRobot robot = makeRobot();
  Move move({.linearGains = LINEAR_GAINS, .headingGains = HEADING_GAINS});
  move.configure(24.0, {});
  robot.failLeftEncoderReads(300, 2);
  CHECK(robot.run(move).result == MotionResult::Settled);
}

TEST_CASE("sim: three rejected readings in a row, allowDegraded off: SensorFault") {
  SimRobot robot = makeRobot();
  Move move({.linearGains = LINEAR_GAINS, .headingGains = HEADING_GAINS});
  move.configure(24.0, {});
  robot.failLeftEncoderReads(300, 3);
  const SimRun run = robot.run(move);
  CHECK(run.result == MotionResult::SensorFault);
  CHECK(run.elapsedMs >= 320u);
  CHECK(run.elapsedMs <= 340u);
}

TEST_CASE("sim: three rejected readings in a row, allowDegraded on: carries on on the fallback") {
  SimRobot robot = makeRobot();
  Move move({.linearGains = LINEAR_GAINS, .headingGains = HEADING_GAINS});
  move.configure(24.0, {.allowDegraded = true});
  robot.failLeftEncoderReads(300, 3);
  const SimRun run = robot.run(move);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(run.result == MotionResult::Settled);
  CHECK(run.trace.back().health == OdometryHealth::Degraded);
  CHECK(std::abs(robot.drive().truePose().yInches - 24.0) < 1.0);
}
