// line_fit_test.cpp
// Why: proves the least-squares gain/offset fit (SPEC section 6) recovers known lines.
// Owns: tests for algorithms/math/line_fit.
#include "cardLib/algorithms/math/line_fit.hpp"

#include <array>

#include "doctest.h"

using namespace cardLib;

TEST_CASE("an exact line y = 2x + 1 gives gain 2, offset 1") {
  const std::array<double, 3> raw{1.0, 2.0, 4.0};
  const std::array<double, 3> truth{3.0, 5.0, 9.0};
  const auto fit = fitLine(raw, truth);
  REQUIRE(fit.has_value());
  CHECK(fit->gain == doctest::Approx(2.0));
  CHECK(fit->offset == doctest::Approx(1.0));
}

TEST_CASE("symmetric noise around y = 2x + 1 still gives gain 2, offset 1") {
  const std::array<double, 4> raw{1.0, 1.0, 3.0, 3.0};
  const std::array<double, 4> truth{3.1, 2.9, 7.1, 6.9};
  const auto fit = fitLine(raw, truth);
  REQUIRE(fit.has_value());
  CHECK(fit->gain == doctest::Approx(2.0));
  CHECK(fit->offset == doctest::Approx(1.0));
}

TEST_CASE("fitLine refuses data it can't fit") {
  const std::array<double, 1> onePoint{1.0};
  CHECK_FALSE(fitLine(onePoint, onePoint).has_value());

  const std::array<double, 3> sameRaw{2.0, 2.0, 2.0};
  const std::array<double, 3> truth{1.0, 2.0, 3.0};
  CHECK_FALSE(fitLine(sameRaw, truth).has_value());

  const std::array<double, 2> shorter{1.0, 2.0};
  CHECK_FALSE(fitLine(shorter, truth).has_value());
}
