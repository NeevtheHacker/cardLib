// holonomic_motion_sim_test.cpp
// Why: the SPEC 20.10 motions run whole on a simulated asterisk drive: strafe, holonomic
//      moveToPoint and moveToPose, and a pivot about a corner (SPEC 20.9).
// Owns: the holonomic sim tests. Gains suit the simulator only; real gains come from tuning.
#include <cmath>
#include <numbers>

#include "cardLib/algorithms/motion/holonomic_move_to_point.hpp"
#include "cardLib/algorithms/motion/holonomic_move_to_pose.hpp"
#include "cardLib/algorithms/motion/pivot_turn.hpp"
#include "cardLib/algorithms/motion/strafe.hpp"
#include "doctest.h"
#include "holonomic_sim.hpp"

using namespace cardLib;
using namespace cardLib::sim;

namespace {

constexpr double CORNER_TOP = std::numbers::pi * 2.75 * 450.0 / 60.0;
constexpr double CENTER_TOP = std::numbers::pi * 2.75 * 600.0 / 60.0;
constexpr double HALF_WIDTH = 5.5;  // example positions only
constexpr double HALF_LENGTH = 5.0;

constexpr DriveKinematics ASTERISK{
    {-HALF_WIDTH, HALF_LENGTH, 45.0, CORNER_TOP},
    {HALF_WIDTH, HALF_LENGTH, -45.0, CORNER_TOP},
    {-HALF_WIDTH, -HALF_LENGTH, -45.0, CORNER_TOP},
    {HALF_WIDTH, -HALF_LENGTH, 45.0, CORNER_TOP},
    {-6.0, 0.0, 0.0, CENTER_TOP},
    {6.0, 0.0, 0.0, CENTER_TOP},
};

constexpr SlewRates LINEAR_SLEW{.accelPerSecond = 500.0, .decelPerSecond = 1500.0};
constexpr SlewRates ANGULAR_SLEW{.accelPerSecond = 1500.0, .decelPerSecond = 3000.0};
constexpr PidGains TRANSLATION_GAINS{.kP = 8.0, .kD = 0.6};
constexpr PidGains HEADING_GAINS{.kP = 2.0};
constexpr PidGains TURN_GAINS{.kP = 2.0, .kD = 0.2};
constexpr std::uint32_t SETTLE_IDLE_MS = 500;

HolonomicSimRobot makeRobot(const Pose& start = {}, bool horizontalWheel = true) {
  return HolonomicSimRobot({.kinematics = ASTERISK, .horizontalWheel = horizontalWheel},
                           LINEAR_SLEW, LINEAR_SLEW, ANGULAR_SLEW, start);
}

double distanceTo(const Pose& pose, double x, double y) {
  return std::hypot(pose.xInches - x, pose.yInches - y);
}

}  // namespace

TEST_CASE("holonomic sim: strafe(24) slides right 24 in and keeps facing forward") {
  HolonomicSimRobot robot = makeRobot();
  Strafe strafe({.strafeGains = TRANSLATION_GAINS, .headingGains = HEADING_GAINS});
  strafe.configure(24.0, {});
  const SimRun run = robot.run(strafe);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(run.result == MotionResult::Settled);
  CHECK(std::abs(robot.truePose().xInches - 24.0) < 1.0);
  CHECK(std::abs(robot.truePose().yInches) < 0.5);
  CHECK(std::abs(robot.truePose().headingDegrees) < 1.0);
}

TEST_CASE("holonomic sim: strafe(-12) at heading 90 slides toward +y") {
  HolonomicSimRobot robot = makeRobot({0.0, 0.0, 90.0});
  Strafe strafe({.strafeGains = TRANSLATION_GAINS, .headingGains = HEADING_GAINS});
  strafe.configure(-12.0, {});
  CHECK(robot.run(strafe).result == MotionResult::Settled);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(std::abs(robot.truePose().yInches - 12.0) < 1.0);
  CHECK(std::abs(robot.truePose().xInches) < 0.5);
}

TEST_CASE("holonomic sim: moveToPoint drives straight at the point and holds its heading") {
  HolonomicSimRobot robot = makeRobot();
  HolonomicMoveToPoint move({.translationGains = TRANSLATION_GAINS, .headingGains = HEADING_GAINS});
  move.configure({24.0, 24.0}, {});
  const SimRun run = robot.run(move);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(run.result == MotionResult::Settled);
  CHECK(distanceTo(robot.truePose(), 24.0, 24.0) < 1.0);
  CHECK(std::abs(robot.truePose().headingDegrees) < 2.0);
  for (const TraceSample& sample : run.trace) {
    // A straight path: never far from the line y = x.
    CHECK(std::abs(sample.truePose.xInches - sample.truePose.yInches) < 2.0);
  }
}

TEST_CASE("holonomic sim: moveToPoint with headingDegrees turns to it on the way") {
  HolonomicSimRobot robot = makeRobot();
  HolonomicMoveToPoint move({.translationGains = TRANSLATION_GAINS, .headingGains = HEADING_GAINS});
  move.configure({-12.0, 18.0}, {.headingDegrees = 90.0});
  CHECK(robot.run(move).result == MotionResult::Settled);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(distanceTo(robot.truePose(), -12.0, 18.0) < 1.0);
  CHECK(std::abs(robot.truePose().headingDegrees - 90.0) < 3.0);
}

TEST_CASE("holonomic sim: moveToPose reaches the point and the heading together") {
  HolonomicSimRobot robot = makeRobot();
  HolonomicMoveToPose move(
      {.drive = {.translationGains = TRANSLATION_GAINS, .headingGains = TURN_GAINS}});
  move.configure({-12.0, 20.0, 135.0}, {});
  const SimRun run = robot.run(move);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(run.result == MotionResult::Settled);
  CHECK(distanceTo(robot.truePose(), -12.0, 20.0) < 1.0);
  CHECK(std::abs(robot.truePose().headingDegrees - 135.0) < 2.0);
}

TEST_CASE("holonomic sim: a pivot about the front-left wheel keeps that wheel in place") {
  HolonomicSimRobot robot = makeRobot();
  const LocalOffset corner{.rightInches = -HALF_WIDTH, .forwardInches = HALF_LENGTH};
  const Point cornerAtStart = robot.truePointOnRobot(corner);
  PivotTurn pivot({.pivotGains = TURN_GAINS});
  pivot.configure(90.0, corner, {});
  CHECK(robot.run(pivot).result == MotionResult::Settled);
  robot.idle(SETTLE_IDLE_MS);
  const Point cornerAtEnd = robot.truePointOnRobot(corner);
  CHECK(std::abs(robot.truePose().headingDegrees - 90.0) < 2.0);
  CHECK(std::hypot(cornerAtEnd.xInches - cornerAtStart.xInches,
                   cornerAtEnd.yInches - cornerAtStart.yInches) < 1.0);
}

TEST_CASE("holonomic sim: no horizontal wheel, strafe tracked from the drive encoders") {
  HolonomicSimRobot robot = makeRobot({}, false);
  Strafe strafe({.strafeGains = TRANSLATION_GAINS, .headingGains = HEADING_GAINS});
  strafe.configure(18.0, {.allowDegraded = true});
  CHECK(robot.run(strafe).result == MotionResult::Settled);
  robot.idle(SETTLE_IDLE_MS);
  CHECK(robot.state().health == OdometryHealth::Degraded);
  CHECK(std::abs(robot.truePose().xInches - 18.0) < 1.0);
  CHECK(std::abs(robot.state().pose.xInches - robot.truePose().xInches) < 0.5);
}

TEST_CASE("holonomic sim: no horizontal wheel without allowDegraded refuses to run") {
  HolonomicSimRobot robot = makeRobot({}, false);
  Strafe strafe({.strafeGains = TRANSLATION_GAINS, .headingGains = HEADING_GAINS});
  strafe.configure(18.0, {});
  robot.idle(20);
  CHECK(robot.run(strafe).result == MotionResult::SensorFault);
}
