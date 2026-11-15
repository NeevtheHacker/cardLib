// drive_pipeline_test.cpp
// Why: proves SPEC section 7 steps 2 to 6 run in order: clamp, minSpeed, slew, mix, millivolts.
// Owns: tests for algorithms/control/drive_pipeline.
#include "cardLib/algorithms/control/drive_pipeline.hpp"

#include "doctest.h"

using namespace cardLib;

namespace {
constexpr double DT = 0.01;
constexpr SlewRates NO_SLEW{};
constexpr ChannelRequest STILL{};
}  // namespace

TEST_CASE("each channel is clamped to its maxSpeed") {
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  const DriveCommand command =
      pipeline.update({.speed = 200.0, .error = 50.0, .limits = {.maxSpeed = 80.0}}, STILL, DT);
  CHECK(command.leftMillivolts == 7559);
  CHECK(command.rightMillivolts == 7559);
}

TEST_CASE("minSpeed bumps a small output while the error is outside the cutoff") {
  const SpeedLimits limits{.maxSpeed = 127.0, .minSpeed = 20.0, .minSpeedCutoff = 1.0};
  CHECK(limitSpeed({.speed = 2.0, .error = 10.0, .limits = limits}) == doctest::Approx(20.0));
  CHECK(limitSpeed({.speed = 50.0, .error = 10.0, .limits = limits}) == doctest::Approx(50.0));
}

TEST_CASE("minSpeed takes its sign from the error, not the output") {
  const SpeedLimits limits{.maxSpeed = 127.0, .minSpeed = 20.0, .minSpeedCutoff = 1.0};
  CHECK(limitSpeed({.speed = 0.0, .error = -10.0, .limits = limits}) == doctest::Approx(-20.0));
  CHECK(limitSpeed({.speed = 0.0, .error = 10.0, .limits = limits}) == doctest::Approx(20.0));
}

TEST_CASE("minSpeed is ignored inside the cutoff, so it can't hunt around the target") {
  const SpeedLimits limits{.maxSpeed = 127.0, .minSpeed = 20.0, .minSpeedCutoff = 1.0};
  CHECK(limitSpeed({.speed = 2.0, .error = 0.5, .limits = limits}) == doctest::Approx(2.0));
  CHECK(limitSpeed({.speed = 2.0, .error = 1.0, .limits = limits}) == doctest::Approx(2.0));
}

TEST_CASE("each channel is slewed before mixing") {
  DrivePipeline pipeline({.accelPerSecond = 500.0}, {.accelPerSecond = 500.0});
  const DriveCommand command =
      pipeline.update({.speed = 127.0, .error = 50.0}, {.speed = 127.0, .error = 50.0}, DT);
  CHECK(pipeline.linearSpeed() == doctest::Approx(5.0));
  CHECK(pipeline.angularSpeed() == doctest::Approx(5.0));
  CHECK(command.leftMillivolts == 945);
  CHECK(command.rightMillivolts == 0);
}

TEST_CASE("reset seeds the slew so chained motions keep moving") {
  DrivePipeline pipeline({.accelPerSecond = 500.0}, {.accelPerSecond = 500.0});
  pipeline.reset(60.0, 0.0);
  pipeline.update({.speed = 127.0, .error = 50.0}, STILL, DT);
  CHECK(pipeline.linearSpeed() == doctest::Approx(65.0));
}

TEST_CASE("desaturation keeps the turn and output never exceeds 12000 mV") {
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  const DriveCommand command =
      pipeline.update({.speed = 127.0, .error = 50.0}, {.speed = 127.0, .error = 50.0}, DT);
  CHECK(command.leftMillivolts == 12000);
  CHECK(command.rightMillivolts == -12000);
}

TEST_CASE("pivot with the left side locked: left held at 0, right gets -u") {
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  const DriveCommand command = pipeline.updatePivot({.speed = 63.5, .error = 30.0}, Side::Left, DT);
  CHECK(command.leftMillivolts == 0);
  CHECK(command.leftMode == SideMode::Held);
  CHECK(command.rightMillivolts == -6000);
  CHECK(command.rightMode == SideMode::Driven);
}

TEST_CASE("pivot with the right side locked: left gets +u, right held at 0") {
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  const DriveCommand command =
      pipeline.updatePivot({.speed = 63.5, .error = 30.0}, Side::Right, DT);
  CHECK(command.leftMillivolts == 6000);
  CHECK(command.leftMode == SideMode::Driven);
  CHECK(command.rightMillivolts == 0);
  CHECK(command.rightMode == SideMode::Held);
}

TEST_CASE("pivot turn speed is limited and slewed like the angular channel") {
  DrivePipeline pipeline(NO_SLEW, {.accelPerSecond = 500.0});
  pipeline.reset(40.0, 0.0);
  pipeline.updatePivot({.speed = 127.0, .error = 30.0, .limits = {.maxSpeed = 80.0}}, Side::Right,
                       DT);
  CHECK(pipeline.angularSpeed() == doctest::Approx(5.0));
  CHECK(pipeline.linearSpeed() == doctest::Approx(0.0));
}

TEST_CASE("the slew remembers the speed before desaturation") {
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  pipeline.update({.speed = 100.0, .error = 50.0}, {.speed = 50.0, .error = 50.0}, DT);
  CHECK(pipeline.linearSpeed() == doctest::Approx(100.0));
}
