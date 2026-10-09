// calibration_math_test.cpp
// Why: proves each SPEC section 6 formula recovers known values from synthetic data.
// Owns: tests for algorithms/calibration/calibration_math.
#include "cardLib/algorithms/calibration/calibration_math.hpp"

#include <array>
#include <numbers>

#include "doctest.h"

using namespace cardLib;

namespace {
constexpr double TEN_TURNS_RADIANS = 10.0 * 2.0 * std::numbers::pi;
}

TEST_CASE("IMU scale: 10 turns that the IMU read short recovers k_imu = 1.009") {
  const double measured = 3600.0 / 1.009;
  const auto scale = imuScale(10.0, measured, -measured);
  REQUIRE(scale.has_value());
  CHECK(*scale == doctest::Approx(1.009));
}

TEST_CASE("IMU scale averages the two directions") {
  const auto scale = imuScale(10.0, 3600.0 / 1.01, -3600.0 / 1.0);
  REQUIRE(scale.has_value());
  CHECK(*scale == doctest::Approx(1.005));
  CHECK_FALSE(imuScale(10.0, 0.0, -3600.0).has_value());
}

TEST_CASE("tracking wheel diameter: D_true = D_nom * L / d_measured") {
  const auto diameter = trackingWheelDiameter(2.75, 72.0, 70.0);
  REQUIRE(diameter.has_value());
  CHECK(*diameter == doctest::Approx(2.75 * 72.0 / 70.0));
  CHECK_FALSE(trackingWheelDiameter(2.75, 72.0, 0.0).has_value());
}

TEST_CASE("drive scale: s_drive = L / d_drive") {
  const auto scale = driveScale(72.0, 72.0 / 0.987);
  REQUIRE(scale.has_value());
  CHECK(*scale == doctest::Approx(0.987));
}

TEST_CASE("wheel offsets come back from a synthetic clockwise spin") {
  // In a pure spin each wheel moves only from turning: dv = -s_v * dtheta, dh = s_h * dtheta.
  const double leftX = -1.5;
  const double rightX = 6.0;
  const double horizontalY = -2.25;
  CHECK(*verticalWheelOffset(-leftX * TEN_TURNS_RADIANS, TEN_TURNS_RADIANS) ==
        doctest::Approx(leftX));
  CHECK(*verticalWheelOffset(-rightX * TEN_TURNS_RADIANS, TEN_TURNS_RADIANS) ==
        doctest::Approx(rightX));
  CHECK(*horizontalWheelOffset(horizontalY * TEN_TURNS_RADIANS, TEN_TURNS_RADIANS) ==
        doctest::Approx(horizontalY));
  CHECK_FALSE(verticalWheelOffset(1.0, 0.0).has_value());
}

TEST_CASE("effective track width from a synthetic spin: (dL - dR) / dtheta") {
  const double trackWidth = 12.1;
  const double left = trackWidth / 2.0 * TEN_TURNS_RADIANS;
  const double right = -trackWidth / 2.0 * TEN_TURNS_RADIANS;
  CHECK(*effectiveTrackWidth(left, right, TEN_TURNS_RADIANS) == doctest::Approx(trackWidth));
  CHECK(*effectiveTrackWidth(-left, -right, -TEN_TURNS_RADIANS) == doctest::Approx(trackWidth));
}

TEST_CASE("median of three") {
  CHECK(medianOfThree(3.0, 1.0, 2.0) == doctest::Approx(2.0));
  CHECK(medianOfThree(1.0, 1.0, 9.0) == doctest::Approx(1.0));
}

TEST_CASE("pass criteria: scales within 0.5 %, offsets within 0.1 in") {
  CHECK(scalesAgree(1.000, 1.004));
  CHECK_FALSE(scalesAgree(1.000, 1.006));
  CHECK(offsetsAgree(-1.50, -1.42));
  CHECK_FALSE(offsetsAgree(-1.50, -1.35));
}

TEST_CASE("tape box test reports position and heading error") {
  const TapeBoxError error = tapeBoxError({0.0, 0.0, 0.0}, {3.0, 4.0, 358.0});
  CHECK(error.positionInches == doctest::Approx(5.0));
  CHECK(error.headingDegrees == doctest::Approx(-2.0));
}

TEST_CASE("UMBmark score is the larger of the clockwise and counterclockwise centroid errors") {
  const std::array<Point, 2> clockwise{{{1.0, 1.0}, {3.0, 3.0}}};
  const std::array<Point, 2> counterclockwise{{{-1.0, 0.0}, {-1.0, 0.0}}};
  const auto score = umbmarkScore(clockwise, counterclockwise);
  REQUIRE(score.has_value());
  CHECK(*score == doctest::Approx(std::numbers::sqrt2 * 2.0));
  CHECK_FALSE(umbmarkScore({}, counterclockwise).has_value());
}
