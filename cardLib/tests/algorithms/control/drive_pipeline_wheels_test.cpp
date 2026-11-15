// drive_pipeline_wheels_test.cpp
// Why: proves the pipeline's SPEC 20 additions: per-wheel commands, the strafe channel and its
//      slew, generalized pivots, and the side summary the screen shows.
// Owns: tests for the per-wheel parts of algorithms/control/drive_pipeline.
#include <cmath>
#include <cstdint>
#include <numbers>

#include "cardLib/algorithms/control/drive_kinematics.hpp"
#include "cardLib/algorithms/control/drive_mixer.hpp"
#include "cardLib/algorithms/control/drive_pipeline.hpp"
#include "doctest.h"

using namespace cardLib;

namespace {

constexpr double DT = 0.01;
constexpr SlewRates NO_SLEW{};
constexpr double CORNER_TOP = std::numbers::pi * 2.75 * 450.0 / 60.0;
constexpr double CENTER_TOP = std::numbers::pi * 2.75 * 600.0 / 60.0;

// Example positions only.
constexpr DriveKinematics ASTERISK{
    {-5.5, 5.0, 45.0, CORNER_TOP}, {5.5, 5.0, -45.0, CORNER_TOP}, {-5.5, -5.0, -45.0, CORNER_TOP},
    {5.5, -5.0, 45.0, CORNER_TOP}, {-6.0, 0.0, 0.0, CENTER_TOP},  {6.0, 0.0, 0.0, CENTER_TOP},
};

constexpr ChannelRequest request(double speed) {
  return {.speed = speed, .error = 50.0};
}

}  // namespace

TEST_CASE("the two-rate constructor drives a unit tank and fills both wheel commands") {
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  const DriveCommand command = pipeline.update(request(30.0), request(20.0), DT);
  CHECK(command.wheelCount == 2);
  CHECK(command.wheels[0].millivolts == command.leftMillivolts);
  CHECK(command.wheels[1].millivolts == command.rightMillivolts);
  CHECK(command.leftMillivolts == speedToMillivolts(50.0));
  CHECK(command.rightMillivolts == speedToMillivolts(10.0));
}

TEST_CASE("a tank ignores the strafe channel") {
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  const DriveCommand command =
      pipeline.updateHolonomic(request(0.0), request(100.0), request(0.0), DT);
  CHECK(command.leftMillivolts == 0);
  CHECK(command.rightMillivolts == 0);
}

TEST_CASE("strafe right on an asterisk drives the corners in the X pattern") {
  DrivePipeline pipeline(ASTERISK, NO_SLEW, NO_SLEW, NO_SLEW);
  const DriveCommand command =
      pipeline.updateHolonomic(request(0.0), request(127.0), request(0.0), DT);
  CHECK(command.wheelCount == 6);
  CHECK(command.wheels[0].millivolts == 12000);
  CHECK(command.wheels[1].millivolts == -12000);
  CHECK(command.wheels[2].millivolts == -12000);
  CHECK(command.wheels[3].millivolts == 12000);
  CHECK(command.wheels[4].millivolts == 0);
  CHECK(command.wheels[5].millivolts == 0);
}

TEST_CASE("the strafe channel has its own slew limiter") {
  DrivePipeline pipeline(ASTERISK, NO_SLEW, {.accelPerSecond = 500.0}, NO_SLEW);
  pipeline.updateHolonomic(request(127.0), request(127.0), request(0.0), DT);
  CHECK(pipeline.linearSpeed() == doctest::Approx(127.0));
  CHECK(pipeline.strafeSpeed() == doctest::Approx(5.0));
}

TEST_CASE("reset seeds the strafe slew too, for chained holonomic motions") {
  DrivePipeline pipeline(ASTERISK, NO_SLEW, {.accelPerSecond = 500.0}, NO_SLEW);
  pipeline.reset(0.0, 0.0, 60.0);
  pipeline.updateHolonomic(request(0.0), request(127.0), request(0.0), DT);
  CHECK(pipeline.strafeSpeed() == doctest::Approx(65.0));
}

TEST_CASE("the side summary is the mean of each side's wheels") {
  DrivePipeline pipeline(ASTERISK, NO_SLEW, NO_SLEW, NO_SLEW);
  const DriveCommand command = pipeline.update(request(127.0), request(0.0), DT);
  const double leftMean =
      (command.wheels[0].millivolts + command.wheels[2].millivolts + command.wheels[4].millivolts) /
      3.0;
  CHECK(command.leftMillivolts == static_cast<std::int32_t>(std::lround(leftMean)));
  CHECK(command.wheels[4].millivolts == 12000);
  CHECK(command.wheels[0].millivolts == speedToMillivolts(127.0 * ASTERISK.forwardDuty(0)));
}

TEST_CASE("a pivot on a real tank gives the same command as on the unit tank") {
  DrivePipeline unit(NO_SLEW, NO_SLEW);
  DrivePipeline real(tankKinematics(12.25, 70.0), NO_SLEW, NO_SLEW, NO_SLEW);
  for (const Side side : {Side::Left, Side::Right}) {
    const DriveCommand a = unit.updatePivot(request(63.5), side, DT);
    const DriveCommand b = real.updatePivot(request(63.5), side, DT);
    CHECK(a.leftMillivolts == b.leftMillivolts);
    CHECK(a.rightMillivolts == b.rightMillivolts);
    CHECK(a.leftMode == b.leftMode);
    CHECK(a.rightMode == b.rightMode);
  }
}

TEST_CASE("pivoting about a point holds the wheels that don't move there") {
  DrivePipeline pipeline(ASTERISK, NO_SLEW, NO_SLEW, NO_SLEW);
  const DriveCommand command =
      pipeline.updatePivotAbout(request(60.0), {.rightInches = -5.5, .forwardInches = 5.0}, DT);
  CHECK(command.wheels[0].mode == SideMode::Held);
  CHECK(command.wheels[0].millivolts == 0);
  CHECK(command.wheels[3].mode == SideMode::Driven);
  CHECK(pipeline.linearSpeed() == 0.0);
  CHECK(pipeline.strafeSpeed() == 0.0);
  CHECK(pipeline.angularSpeed() == doctest::Approx(60.0));
}

TEST_CASE("setKinematics swaps the drive the pipeline mixes for") {
  DrivePipeline pipeline(NO_SLEW, NO_SLEW);
  pipeline.setKinematics(ASTERISK);
  CHECK(pipeline.kinematics().wheelCount() == 6);
  const DriveCommand command = pipeline.update(request(127.0), request(0.0), DT);
  CHECK(command.wheelCount == 6);
}

TEST_CASE("toDriveCommand turns wheel speeds into millivolts and held modes") {
  WheelSpeeds wheels;
  wheels.count = 2;
  wheels.speeds = {63.5, 0.0};
  wheels.held = {false, true};
  const DriveCommand command = toDriveCommand(wheels, UNIT_TANK_KINEMATICS);
  CHECK(command.wheels[0].millivolts == 6000);
  CHECK(command.wheels[1].mode == SideMode::Held);
  CHECK(command.leftMillivolts == 6000);
  CHECK(command.rightMode == SideMode::Held);
}

TEST_CASE("summarizeSides fills the side summary of a raw per-wheel command") {
  DriveCommand command;
  command.wheelCount = 6;
  const std::int32_t raw[6] = {3000, -1000, 2000, 0, 4000, 6000};
  for (std::size_t i = 0; i < 6; ++i) {
    command.wheels[i].millivolts = raw[i];
  }
  summarizeSides(command, ASTERISK);
  CHECK(command.leftMillivolts == 3000);   // wheels 0, 2, 4: (3000 + 2000 + 4000) / 3
  CHECK(command.rightMillivolts == 1667);  // wheels 1, 3, 5: (-1000 + 0 + 6000) / 3, rounded
  CHECK(command.leftMode == SideMode::Driven);
  CHECK(command.rightMode == SideMode::Driven);
}

TEST_CASE("summarizeSides: one wheel alone shows on its own side only") {
  DriveCommand command;
  command.wheelCount = 6;
  command.wheels[3].millivolts = 6000;
  summarizeSides(command, ASTERISK);
  CHECK(command.leftMillivolts == 0);
  CHECK(command.rightMillivolts == 2000);
}

TEST_CASE("drivenMillivoltMagnitude averages only the driven wheels") {
  DriveCommand command;
  command.wheelCount = 4;
  const std::int32_t raw[4] = {6000, -6000, 0, 6000};
  for (std::size_t i = 0; i < 4; ++i) {
    command.wheels[i].millivolts = raw[i];
  }
  command.wheels[2].mode = SideMode::Held;
  CHECK(drivenMillivoltMagnitude(command) == doctest::Approx(6000.0));
  command.wheels[2].mode = SideMode::Driven;
  CHECK(drivenMillivoltMagnitude(command) == doctest::Approx(4500.0));
}

TEST_CASE("drivenMillivoltMagnitude on a two-side command skips a held side") {
  DriveCommand command{.leftMillivolts = 8000, .rightMillivolts = 0, .rightMode = SideMode::Held};
  CHECK(drivenMillivoltMagnitude(command) == doctest::Approx(8000.0));
  command.rightMode = SideMode::Driven;
  command.rightMillivolts = -4000;
  CHECK(drivenMillivoltMagnitude(command) == doctest::Approx(6000.0));
}

TEST_CASE("drivenMillivoltMagnitude is 0 when nothing is driven") {
  DriveCommand command{.leftMode = SideMode::Held, .rightMode = SideMode::Held};
  CHECK(drivenMillivoltMagnitude(command) == 0.0);
  command.wheelCount = 2;
  command.wheels[0].mode = SideMode::Held;
  command.wheels[1].mode = SideMode::Held;
  CHECK(drivenMillivoltMagnitude(command) == 0.0);
}
