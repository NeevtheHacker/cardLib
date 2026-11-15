// drive_kinematics_test.cpp
// Why: proves one wheel-module model covers tank and holonomic drives (SPEC 20): rows, top
//      speeds, speed matching, desaturation with rotation priority, pivots and least-squares
//      forward kinematics.
// Owns: tests for algorithms/control/drive_kinematics.
#include "cardLib/algorithms/control/drive_kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include "doctest.h"

using namespace cardLib;

namespace {

constexpr double topSpeed(double diameterInches, double wheelRpm) {
  return std::numbers::pi * diameterInches * wheelRpm / 60.0;
}

// The SPEC 20.6 worked example: 2.75 in omnis everywhere, corners 36:48 on blue (450 rpm) at
// 45 degrees, centers 1:1 on blue (600 rpm). Positions are example values only.
constexpr double CORNER_HALF_WIDTH = 5.5;
constexpr double CORNER_HALF_LENGTH = 5.0;
constexpr double CENTER_HALF_WIDTH = 6.0;
constexpr double CORNER_TOP = topSpeed(2.75, 600.0 * 36.0 / 48.0);
constexpr double CENTER_TOP = topSpeed(2.75, 600.0);

constexpr DriveKinematics ASTERISK{
    {-CORNER_HALF_WIDTH, CORNER_HALF_LENGTH, 45.0, CORNER_TOP},    // front left
    {CORNER_HALF_WIDTH, CORNER_HALF_LENGTH, -45.0, CORNER_TOP},    // front right
    {-CORNER_HALF_WIDTH, -CORNER_HALF_LENGTH, -45.0, CORNER_TOP},  // back left
    {CORNER_HALF_WIDTH, -CORNER_HALF_LENGTH, 45.0, CORNER_TOP},    // back right
    {-CENTER_HALF_WIDTH, 0.0, 0.0, CENTER_TOP},                    // center left
    {CENTER_HALF_WIDTH, 0.0, 0.0, CENTER_TOP},                     // center right
};

constexpr SpeedMatchReport ASTERISK_REPORT = speedMatchReport(ASTERISK);
static_assert(requireSpeedMatchWithin(ASTERISK_REPORT, 0.10));
static_assert(!requireSpeedMatchWithin(ASTERISK_REPORT, 0.05));
static_assert(UNIT_TANK_KINEMATICS.maxSpinRadiansPerSecond() == 2.0);
static_assert(!UNIT_TANK_KINEMATICS.canStrafe());

// The SPEC section 7 mixer as it was before kinematics: clamp ang, shrink lin, mix.
std::array<double, 2> sectionSevenMix(double linear, double angular) {
  const double clampedAngular = std::clamp(angular, -127.0, 127.0);
  if (std::abs(linear) + std::abs(clampedAngular) > 127.0) {
    linear = std::copysign(127.0 - std::abs(clampedAngular), linear);
  }
  return {linear + clampedAngular, linear - clampedAngular};
}

double largestMagnitude(const WheelSpeeds& wheels) {
  double largest = 0.0;
  for (std::size_t i = 0; i < wheels.count; ++i) {
    largest = std::max(largest, std::abs(wheels.speeds[i]));
  }
  return largest;
}

// Each wheel's surface travel for a robot motion (SPEC 20.5), for forward-kinematics tests.
std::array<double, MAX_WHEEL_MODULES> surfaceTravel(const DriveKinematics& kinematics, double right,
                                                    double forward, double turn) {
  std::array<double, MAX_WHEEL_MODULES> travel = {};
  for (std::size_t i = 0; i < kinematics.wheelCount(); ++i) {
    const WheelRow& row = kinematics.surfaceRow(i);
    travel[i] = row.strafe * right + row.forward * forward + row.turn * turn;
  }
  return travel;
}

constexpr std::array<bool, MAX_WHEEL_MODULES> ALL_USABLE{true, true, true, true,
                                                         true, true, true, true};

}  // namespace

TEST_CASE("a tank's rows are (0, 1, W/2) and (0, 1, -W/2) over its top speed") {
  const DriveKinematics tank = tankKinematics(12.0, 60.0);
  CHECK(tank.wheelCount() == 2);
  CHECK(tank.surfaceRow(0).forward == 1.0);
  CHECK(tank.surfaceRow(0).strafe == 0.0);
  CHECK(tank.surfaceRow(0).turn == doctest::Approx(6.0));
  CHECK(tank.surfaceRow(1).turn == doctest::Approx(-6.0));
  CHECK(tank.normalizedRow(0).turn == doctest::Approx(6.0 / 60.0));
  CHECK(tank.maxForwardInchesPerSecond() == doctest::Approx(60.0));
  CHECK(tank.maxStrafeInchesPerSecond() == 0.0);
  CHECK(tank.maxSpinRadiansPerSecond() == doctest::Approx(10.0));
  CHECK_FALSE(tank.canStrafe());
}

TEST_CASE("on a tank, desaturation is exactly the section 7 mixer") {
  const DriveKinematics realTank = tankKinematics(12.25, topSpeed(3.25, 450.0));
  for (double linear = -200.0; linear <= 200.0; linear += 25.0) {
    for (double angular = -200.0; angular <= 200.0; angular += 25.0) {
      const std::array<double, 2> expected = sectionSevenMix(linear, angular);
      const WheelSpeeds unit =
          UNIT_TANK_KINEMATICS.wheelSpeeds({.forward = linear, .turn = angular});
      const WheelSpeeds real = realTank.wheelSpeeds({.forward = linear, .turn = angular});
      CHECK(unit.speeds[0] == doctest::Approx(expected[0]).epsilon(1e-12));
      CHECK(unit.speeds[1] == doctest::Approx(expected[1]).epsilon(1e-12));
      CHECK(real.speeds[0] == doctest::Approx(expected[0]).epsilon(1e-9));
      CHECK(real.speeds[1] == doctest::Approx(expected[1]).epsilon(1e-9));
    }
  }
}

TEST_CASE("asterisk worked example: 86.4 in/s forward, corner duty 0.943, 91.6 in/s strafe") {
  CHECK(ASTERISK_REPORT.maxForwardInchesPerSecond == doctest::Approx(86.4).epsilon(0.001));
  CHECK(ASTERISK_REPORT.forwardDuty[0] == doctest::Approx(0.943).epsilon(0.001));
  CHECK(ASTERISK_REPORT.forwardDuty[3] == doctest::Approx(0.943).epsilon(0.001));
  CHECK(ASTERISK_REPORT.forwardDuty[4] == doctest::Approx(1.0));
  CHECK(ASTERISK_REPORT.maxStrafeInchesPerSecond == doctest::Approx(91.6).epsilon(0.001));
  CHECK(ASTERISK_REPORT.mismatch == doctest::Approx(0.057).epsilon(0.02));
  CHECK(CORNER_TOP == doctest::Approx(64.8).epsilon(0.001));
  CHECK(CENTER_TOP == doctest::Approx(86.4).epsilon(0.001));
}

TEST_CASE("full forward runs the centers at 127 and the corners at 94.3 percent") {
  const WheelSpeeds wheels = ASTERISK.wheelSpeeds({.forward = 127.0});
  for (std::size_t i = 0; i < 4; ++i) {
    CHECK(wheels.speeds[i] == doctest::Approx(127.0 * 0.943).epsilon(0.001));
  }
  CHECK(wheels.speeds[4] == doctest::Approx(127.0));
  CHECK(wheels.speeds[5] == doctest::Approx(127.0));
}

TEST_CASE("corner standard: strafe right is FL+, FR-, BL-, BR+ and the centers rest") {
  const WheelSpeeds wheels = ASTERISK.wheelSpeeds({.strafe = 100.0});
  CHECK(wheels.speeds[0] > 0.0);
  CHECK(wheels.speeds[1] < 0.0);
  CHECK(wheels.speeds[2] < 0.0);
  CHECK(wheels.speeds[3] > 0.0);
  CHECK(wheels.speeds[4] == doctest::Approx(0.0));
  CHECK(wheels.speeds[5] == doctest::Approx(0.0));
}

TEST_CASE("corner standard: a clockwise turn is every left wheel + and every right wheel -") {
  const WheelSpeeds wheels = ASTERISK.wheelSpeeds({.turn = 80.0});
  for (const std::size_t left : {0u, 2u, 4u}) {
    CHECK(wheels.speeds[left] > 0.0);
  }
  for (const std::size_t right : {1u, 3u, 5u}) {
    CHECK(wheels.speeds[right] < 0.0);
  }
  CHECK(largestMagnitude(wheels) == doctest::Approx(80.0));
}

TEST_CASE("rotation priority: translation shrinks, the turn stays whole") {
  const DriveSpeeds turnOnly{.turn = 60.0};
  const WheelSpeeds turning = ASTERISK.wheelSpeeds(turnOnly);
  const WheelSpeeds translating = ASTERISK.wheelSpeeds({.forward = 127.0, .strafe = 50.0});
  const WheelSpeeds both = ASTERISK.wheelSpeeds({.forward = 127.0, .strafe = 50.0, .turn = 60.0});
  CHECK(largestMagnitude(both) == doctest::Approx(127.0));
  // both = k * translation + rotation, with one k for every wheel.
  const double k = (both.speeds[4] - turning.speeds[4]) / translating.speeds[4];
  CHECK(k > 0.0);
  CHECK(k < 1.0);
  for (std::size_t i = 0; i < both.count; ++i) {
    CHECK(both.speeds[i] == doctest::Approx(k * translating.speeds[i] + turning.speeds[i]));
  }
}

TEST_CASE("a turn bigger than any wheel can make is scaled down first") {
  const WheelSpeeds wheels = ASTERISK.wheelSpeeds({.forward = 100.0, .turn = 300.0});
  const WheelSpeeds fullTurn = ASTERISK.wheelSpeeds({.turn = 127.0});
  for (std::size_t i = 0; i < wheels.count; ++i) {
    CHECK(wheels.speeds[i] == doctest::Approx(fullTurn.speeds[i]));
  }
  CHECK(largestMagnitude(wheels) == doctest::Approx(127.0));
}

TEST_CASE("no wheel ever goes past 127") {
  for (double forward = -150.0; forward <= 150.0; forward += 50.0) {
    for (double strafe = -150.0; strafe <= 150.0; strafe += 50.0) {
      for (double turn = -150.0; turn <= 150.0; turn += 50.0) {
        const WheelSpeeds wheels =
            ASTERISK.wheelSpeeds({.forward = forward, .strafe = strafe, .turn = turn});
        CHECK(largestMagnitude(wheels) <= 127.0 + 1e-9);
      }
    }
  }
}

TEST_CASE("forward kinematics on a tank is the (L - R) / W heading fallback") {
  const DriveKinematics tank = tankKinematics(12.0, 60.0);
  const std::array<double, 2> travel{3.0, 1.0};
  const std::array<bool, 2> usable{true, true};
  const BodyMotion motion = tank.bodyMotion(travel, usable);
  CHECK(motion.valid);
  CHECK_FALSE(motion.rightKnown);
  CHECK(motion.forwardInches == doctest::Approx(2.0));
  CHECK(motion.headingRadians == doctest::Approx((3.0 - 1.0) / 12.0));
  CHECK(motion.rightInches == 0.0);
  CHECK(motion.residualInches == doctest::Approx(0.0));
}

TEST_CASE("forward kinematics recovers an asterisk's motion, sideways included") {
  const auto travel = surfaceTravel(ASTERISK, 1.5, 2.0, 0.1);
  const BodyMotion motion =
      ASTERISK.bodyMotion(std::span<const double>(travel.data(), 6), ALL_USABLE);
  CHECK(motion.valid);
  CHECK(motion.rightKnown);
  CHECK(motion.rightInches == doctest::Approx(1.5));
  CHECK(motion.forwardInches == doctest::Approx(2.0));
  CHECK(motion.headingRadians == doctest::Approx(0.1));
  CHECK(motion.residualInches < 1e-9);
}

TEST_CASE("forward kinematics leaves a faulted wheel out and measures slip in the residual") {
  auto travel = surfaceTravel(ASTERISK, -0.5, 1.0, -0.05);
  std::array<bool, MAX_WHEEL_MODULES> usable = ALL_USABLE;
  usable[2] = false;
  travel[2] = 99.0;  // a faulted wheel's reading must not matter
  const BodyMotion withoutWheel =
      ASTERISK.bodyMotion(std::span<const double>(travel.data(), 6), usable);
  CHECK(withoutWheel.valid);
  CHECK(withoutWheel.rightInches == doctest::Approx(-0.5));
  CHECK(withoutWheel.forwardInches == doctest::Approx(1.0));
  CHECK(withoutWheel.headingRadians == doctest::Approx(-0.05));

  travel[2] = surfaceTravel(ASTERISK, -0.5, 1.0, -0.05)[2] + 0.5;  // that wheel slipped
  const BodyMotion slipping =
      ASTERISK.bodyMotion(std::span<const double>(travel.data(), 6), ALL_USABLE);
  CHECK(slipping.residualInches > 0.1);
}

TEST_CASE("forward kinematics with too few wheels is not valid") {
  const DriveKinematics tank = tankKinematics(12.0, 60.0);
  const std::array<double, 2> travel{3.0, 1.0};
  const std::array<bool, 2> oneWheel{true, false};
  CHECK_FALSE(tank.bodyMotion(travel, oneWheel).valid);
}

TEST_CASE("pivoting about a tank's left wheel holds it and runs the right wheel backward") {
  const DriveKinematics tank = tankKinematics(12.0, 60.0);
  const WheelSpeeds wheels = tank.pivotSpeeds(63.5, {.rightInches = -6.0, .forwardInches = 0.0});
  CHECK(wheels.held[0]);
  CHECK(wheels.speeds[0] == 0.0);
  CHECK_FALSE(wheels.held[1]);
  CHECK(wheels.speeds[1] == doctest::Approx(-63.5));
}

TEST_CASE("a tank can only pivot about points on its axle line; a holonomic drive anywhere") {
  const DriveKinematics tank = tankKinematics(12.0, 60.0);
  CHECK(tank.canPivotAbout({.rightInches = 20.0, .forwardInches = 0.0}));
  CHECK_FALSE(tank.canPivotAbout({.rightInches = 0.0, .forwardInches = 5.0}));
  CHECK(ASTERISK.canPivotAbout({.rightInches = 0.0, .forwardInches = 5.0}));
}

TEST_CASE("pivoting an asterisk about its front-left wheel holds that wheel") {
  const WheelSpeeds wheels = ASTERISK.pivotSpeeds(
      -50.0, {.rightInches = -CORNER_HALF_WIDTH, .forwardInches = CORNER_HALF_LENGTH});
  CHECK(wheels.held[0]);
  for (std::size_t i = 1; i < wheels.count; ++i) {
    CHECK_FALSE(wheels.held[i]);
  }
  CHECK(largestMagnitude(wheels) == doctest::Approx(50.0));
}
