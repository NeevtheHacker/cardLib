// drive_presets_test.cpp
// Why: proves the tank, asterisk and custom presets put each wheel where SPEC 20.3 and 20.4 say,
//      and that the speed-match report works before the code is compiled (SPEC 20.7).
// Owns: tests for chassis/drive_presets (header-only, constexpr; no motor is ever read).
#include "cardLib/chassis/drive_presets.hpp"

#include "doctest.h"

using namespace cardLib;

namespace {

// Never defined: only their addresses are taken, inside a constant expression that is never
// emitted into the program.
extern MotorGroup FAKE_LEFT;
extern MotorGroup FAKE_RIGHT;

constexpr bool tankPresetIsRight() {
  const DriveConfig drive = tank(FAKE_LEFT, FAKE_RIGHT, Omniwheel::NEW_325, {36, 48}, 12.25);
  return drive.moduleCount == 2 && drive.left == &FAKE_LEFT && drive.right == &FAKE_RIGHT &&
         drive.modules[0].motors == &FAKE_LEFT && drive.modules[1].motors == &FAKE_RIGHT &&
         drive.modules[0].xInches == -6.125 && drive.modules[1].xInches == 6.125 &&
         drive.modules[0].gearRatio == 0.75 && drive.modules[0].wheelDiameterInches == 3.25 &&
         drive.modules[0].rollAngleDegrees == 0.0 && drive.modules[1].rollAngleDegrees == 0.0;
}
static_assert(tankPresetIsRight());

// The SPEC 20.6 worked example. Positions are example values only.
constexpr DriveConfig ASTERISK = asterisk({
    .cornerWheel = Omniwheel::NEW_275,
    .cornerGearing = {36, 48},
    .cornerHalfWidthInches = 5.5,
    .cornerHalfLengthInches = 5.0,
    .centerWheel = Omniwheel::NEW_275,
    .centerGearing = {1, 1},
    .centerHalfWidthInches = 6.0,
});
constexpr SpeedMatchReport ASTERISK_SPEED = speedMatch(ASTERISK, Cartridge::Blue600);
static_assert(requireSpeedMatchWithin(ASTERISK_SPEED, 0.10),
              "corner and center wheels disagree by more than 10%");
static_assert(ASTERISK_SPEED.maxForwardInchesPerSecond > 86.3 &&
              ASTERISK_SPEED.maxForwardInchesPerSecond < 86.5);

}  // namespace

TEST_CASE("asterisk preset: 86.4 in/s forward, corner duty 0.943, 91.6 in/s strafe") {
  CHECK(ASTERISK_SPEED.maxForwardInchesPerSecond == doctest::Approx(86.4).epsilon(0.001));
  CHECK(ASTERISK_SPEED.forwardDuty[0] == doctest::Approx(0.943).epsilon(0.001));
  CHECK(ASTERISK_SPEED.maxStrafeInchesPerSecond == doctest::Approx(91.6).epsilon(0.001));
}

TEST_CASE("asterisk preset puts the wheels in the SPEC 20.3 order and angles") {
  CHECK(ASTERISK.moduleCount == 6);
  CHECK(ASTERISK.left == nullptr);
  const double expected[6][3] = {{-5.5, 5.0, 45.0}, {5.5, 5.0, -45.0}, {-5.5, -5.0, -45.0},
                                 {5.5, -5.0, 45.0}, {-6.0, 0.0, 0.0},  {6.0, 0.0, 0.0}};
  for (std::size_t i = 0; i < 6; ++i) {
    CHECK(ASTERISK.modules[i].xInches == expected[i][0]);
    CHECK(ASTERISK.modules[i].yInches == expected[i][1]);
    CHECK(ASTERISK.modules[i].rollAngleDegrees == expected[i][2]);
    CHECK(ASTERISK.modules[i].wheelDiameterInches == 2.75);
  }
  CHECK(ASTERISK.modules[0].gearRatio == 0.75);
  CHECK(ASTERISK.modules[4].gearRatio == 1.0);
}

TEST_CASE("custom drive: an X-drive listed by hand") {
  constexpr DriveConfig xDrive = customDrive({
      {.wheelDiameterInches = 4.0,
       .gearing = 1.0,
       .xInches = -7,
       .yInches = 7,
       .rollAngleDegrees = 45},
      {.wheelDiameterInches = 4.0,
       .gearing = 1.0,
       .xInches = 7,
       .yInches = 7,
       .rollAngleDegrees = -45},
      {.wheelDiameterInches = 4.0,
       .gearing = 1.0,
       .xInches = -7,
       .yInches = -7,
       .rollAngleDegrees = -45},
      {.wheelDiameterInches = 4.0,
       .gearing = 1.0,
       .xInches = 7,
       .yInches = -7,
       .rollAngleDegrees = 45},
  });
  constexpr SpeedMatchReport report = speedMatch(xDrive, Cartridge::Green200);
  static_assert(requireSpeedMatchWithin(report, 0.0));
  CHECK(report.wheelCount == 4);
  // Every wheel at 45 degrees: forward and strafe tops are both sqrt(2) times a wheel's.
  CHECK(report.maxForwardInchesPerSecond == doctest::Approx(report.maxStrafeInchesPerSecond));
}

TEST_CASE("a 5.5 W motor drive is computed at 200 rpm") {
  constexpr DriveConfig small = customDrive({
      {.wheelDiameterInches = 3.25, .gearing = {1, 1}, .xInches = -5, .rollAngleDegrees = 0},
      {.wheelDiameterInches = 3.25, .gearing = {1, 1}, .xInches = 5, .rollAngleDegrees = 0},
  });
  constexpr SpeedMatchReport report = speedMatch(small, Cartridge::Blue600, MotorType::Exp5_5W);
  CHECK(report.maxForwardInchesPerSecond == doctest::Approx(3.14159265 * 3.25 * 200.0 / 60.0));
}
