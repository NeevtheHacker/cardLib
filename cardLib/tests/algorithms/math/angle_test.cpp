// angle_test.cpp
// Why: proves angle wrap, turn error and heading-to-point match SPEC section 1.
// Owns: tests for algorithms/math/angle.
#include "cardLib/algorithms/math/angle.hpp"

#include <cmath>

#include "doctest.h"

using namespace cardLib;

TEST_CASE("wrapDegrees keeps angles in [-180, 180)") {
  CHECK(wrapDegrees(0.0) == doctest::Approx(0.0));
  CHECK(wrapDegrees(190.0) == doctest::Approx(-170.0));
  CHECK(wrapDegrees(-190.0) == doctest::Approx(170.0));
  CHECK(wrapDegrees(540.0) == doctest::Approx(-180.0));
  CHECK(wrapDegrees(720.5) == doctest::Approx(0.5));
  CHECK(wrapDegrees(179.9) == doctest::Approx(179.9));
}

TEST_CASE("wrapDegrees sends exactly +-180 to -180, so a 180 degree turn goes counterclockwise") {
  CHECK(wrapDegrees(180.0) == doctest::Approx(-180.0));
  CHECK(wrapDegrees(-180.0) == doctest::Approx(-180.0));
  CHECK(turnErrorDegrees(180.0, 0.0) == doctest::Approx(-180.0));
}

TEST_CASE("turnErrorDegrees takes the shortest way by default") {
  CHECK(turnErrorDegrees(10.0, 350.0) == doctest::Approx(20.0));
  CHECK(turnErrorDegrees(350.0, 10.0) == doctest::Approx(-20.0));
  CHECK(turnErrorDegrees(90.0, 720.0) == doctest::Approx(90.0));
}

TEST_CASE("turnErrorDegrees honors a forced direction") {
  CHECK(turnErrorDegrees(0.0, 10.0, TurnDirection::Clockwise) == doctest::Approx(350.0));
  CHECK(turnErrorDegrees(10.0, 0.0, TurnDirection::CounterClockwise) == doctest::Approx(-350.0));
  CHECK(turnErrorDegrees(20.0, 0.0, TurnDirection::Clockwise) == doctest::Approx(20.0));
  CHECK(turnErrorDegrees(0.0, 20.0, TurnDirection::CounterClockwise) == doctest::Approx(-20.0));
  CHECK(turnErrorDegrees(180.0, 0.0, TurnDirection::Clockwise) == doctest::Approx(180.0));
}

TEST_CASE("headingToPointDegrees uses 0 = +y, clockwise positive") {
  CHECK(headingToPointDegrees(0.0, 0.0, 0.0, 10.0) == doctest::Approx(0.0));
  CHECK(headingToPointDegrees(0.0, 0.0, 10.0, 0.0) == doctest::Approx(90.0));
  CHECK(headingToPointDegrees(0.0, 0.0, -10.0, 0.0) == doctest::Approx(-90.0));
  CHECK(std::abs(headingToPointDegrees(0.0, 0.0, 0.0, -10.0)) == doctest::Approx(180.0));
  CHECK(headingToPointDegrees(5.0, 5.0, 15.0, 15.0) == doctest::Approx(45.0));
}

TEST_CASE("reverseHeadingDegrees adds 180 and wraps, for driving backwards to a point") {
  CHECK(reverseHeadingDegrees(headingToPointDegrees(0.0, 0.0, 0.0, 10.0)) ==
        doctest::Approx(-180.0));
  CHECK(reverseHeadingDegrees(90.0) == doctest::Approx(-90.0));
  CHECK(reverseHeadingDegrees(-90.0) == doctest::Approx(90.0));
}

TEST_CASE("degree and radian conversions round-trip") {
  CHECK(degreesToRadians(180.0) == doctest::Approx(3.14159265358979));
  CHECK(radiansToDegrees(degreesToRadians(37.5)) == doctest::Approx(37.5));
}
