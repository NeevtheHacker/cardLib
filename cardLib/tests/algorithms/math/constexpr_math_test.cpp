// constexpr_math_test.cpp
// Why: proves the compile-time sin and cos match std::sin and std::cos, and are exact where a
//      tank's rows need them to be.
// Owns: tests for algorithms/math/constexpr_math.
#include "cardLib/algorithms/math/constexpr_math.hpp"

#include <cmath>
#include <initializer_list>
#include <numbers>

#include "doctest.h"

using namespace cardLib;

static_assert(constexprSinDegrees(0.0) == 0.0);
static_assert(constexprCosDegrees(0.0) == 1.0);
static_assert(constexprSinDegrees(90.0) == 1.0);
static_assert(constexprCosDegrees(90.0) == 0.0);
static_assert(constexprSinDegrees(-90.0) == -1.0);
static_assert(constexprSinDegrees(180.0) == 0.0);
static_assert(constexprCosDegrees(180.0) == -1.0);
static_assert(constexprSinDegrees(45.0) == constexprCosDegrees(45.0));
static_assert(constexprAbs(-2.5) == 2.5);

TEST_CASE("constexpr sin and cos match the standard library every 7.5 degrees") {
  for (double degrees = -720.0; degrees <= 720.0; degrees += 7.5) {
    const double radians = degrees * std::numbers::pi / 180.0;
    CHECK(constexprSinDegrees(degrees) == doctest::Approx(std::sin(radians)).epsilon(1e-14));
    CHECK(constexprCosDegrees(degrees) == doctest::Approx(std::cos(radians)).epsilon(1e-14));
  }
}

TEST_CASE("constexpr sin is accurate at awkward angles") {
  for (const double degrees : {1e-6, 0.3, 33.3, 89.999, 135.0, -44.4, 179.9, -179.9, 1000.1}) {
    const double radians = degrees * std::numbers::pi / 180.0;
    CHECK(std::abs(constexprSinDegrees(degrees) - std::sin(radians)) < 1e-13);
    CHECK(std::abs(constexprCosDegrees(degrees) - std::cos(radians)) < 1e-13);
  }
}

TEST_CASE("multiples of 90 degrees are exact") {
  CHECK(constexprSinDegrees(270.0) == -1.0);
  CHECK(constexprSinDegrees(-270.0) == 1.0);
  CHECK(constexprCosDegrees(-180.0) == -1.0);
  CHECK(constexprCosDegrees(360.0) == 1.0);
  CHECK(constexprCosDegrees(-90.0) == 0.0);
}
