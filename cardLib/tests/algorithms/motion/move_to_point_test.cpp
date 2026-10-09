// move_to_point_test.cpp
// Why: proves moveToPoint() uses the single projection (no cos squared), backs toward points
//      behind it, freezes its heading near the target and exits on true distance (SPEC 8).
// Owns: one-step tests for algorithms/motion/move_to_point.
#include "cardLib/algorithms/motion/move_to_point.hpp"

#include "doctest.h"

using namespace cardLib;

namespace {

const MoveToPointSettings SETTINGS{.linearGains = {.kP = 1.0}, .headingGains = {.kP = 0.0}};
constexpr SlewRates NO_SLEW{};

RobotState stateAt(double x, double y, double heading) {
  return {.pose = {x, y, heading}};
}

// Linear speed the pipeline chose this step (heading gain is 0, so both sides match).
double linearSpeedFor(const RobotState& state, const Point& target,
                      const MotionOptions& options = {}) {
  MoveToPoint motion(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure(target, options);
  motion.start(state);
  motion.step(state, {}, pipeline);
  return pipeline.linearSpeed();
}

}  // namespace

TEST_CASE("target 10 in ahead: linear error 10") {
  CHECK(linearSpeedFor(stateAt(0.0, 0.0, 0.0), {0.0, 10.0}) == doctest::Approx(10.0));
}

TEST_CASE("facing 90 degrees off: no linear drive, it turns first") {
  CHECK(linearSpeedFor(stateAt(0.0, 0.0, 90.0), {0.0, 10.0}) ==
        doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("facing 120 degrees off: linear is negative, never drives away (no cos squared)") {
  CHECK(linearSpeedFor(stateAt(0.0, 0.0, 120.0), {0.0, 10.0}) < 0.0);
}

TEST_CASE("backwards: the robot faces away and drives in reverse") {
  MoveToPointSettings settings = SETTINGS;
  settings.headingGains = {.kP = 1.0};
  MoveToPoint motion(settings);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, -10.0}, {.forwards = false});
  motion.start(stateAt(0.0, 0.0, 0.0));
  motion.step(stateAt(0.0, 0.0, 0.0), {}, pipeline);
  CHECK(pipeline.linearSpeed() == doctest::Approx(-10.0));
  CHECK(pipeline.angularSpeed() == doctest::Approx(0.0));
}

TEST_CASE("inside the settle radius the heading target freezes") {
  MoveToPointSettings settings = SETTINGS;
  settings.headingGains = {.kP = 1.0};
  MoveToPoint motion(settings);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, 10.0}, {});
  motion.start(stateAt(0.0, 0.0, 0.0));
  motion.step(stateAt(0.0, 0.0, 0.0), {}, pipeline);
  motion.step(stateAt(3.0, 9.0, 0.0), {.elapsedMs = 10}, pipeline);
  CHECK(pipeline.angularSpeed() == doctest::Approx(0.0));
}

TEST_CASE("outside the settle radius the heading follows the point") {
  MoveToPointSettings settings = SETTINGS;
  settings.headingGains = {.kP = 1.0};
  MoveToPoint motion(settings);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, 20.0}, {});
  motion.start(stateAt(0.0, 0.0, 0.0));
  motion.step(stateAt(10.0, 0.0, 0.0), {}, pipeline);
  CHECK(pipeline.angularSpeed() < 0.0);
}

TEST_CASE("exits use the true distance, so facing sideways near zero e_lin doesn't settle") {
  MoveToPoint motion(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, 10.0}, {.earlyExit = 2.0});
  motion.start(stateAt(0.0, 0.0, 90.0));
  CHECK_FALSE(motion.step(stateAt(0.0, 0.0, 90.0), {}, pipeline).exit.has_value());
  CHECK(motion.step(stateAt(0.0, 8.5, 90.0), {.elapsedMs = 10}, pipeline).exit.has_value());
}

TEST_CASE("progress counts inches actually travelled") {
  MoveToPoint motion(SETTINGS);
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  motion.configure({0.0, 50.0}, {});
  motion.start(stateAt(0.0, 0.0, 0.0));
  motion.step(stateAt(3.0, 4.0, 0.0), {}, pipeline);
  CHECK(motion.step(stateAt(3.0, 10.0, 0.0), {}, pipeline).progress == doctest::Approx(11.0));
}
