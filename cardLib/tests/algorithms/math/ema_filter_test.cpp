// ema_filter_test.cpp
// Why: proves the exponential moving average (SPEC section 4) smooths as specified.
// Owns: tests for algorithms/math/ema_filter.
#include "cardLib/algorithms/math/ema_filter.hpp"

#include "doctest.h"

using namespace cardLib;

TEST_CASE("alpha = 1 passes the input straight through") {
  EmaFilter filter(1.0);
  CHECK(filter.update(10.0) == doctest::Approx(10.0));
  CHECK(filter.update(-3.0) == doctest::Approx(-3.0));
}

TEST_CASE("alpha = 0.5 on a step to 10 gives 5, 7.5, 8.75") {
  EmaFilter filter(0.5);
  CHECK(filter.update(10.0) == doctest::Approx(5.0));
  CHECK(filter.update(10.0) == doctest::Approx(7.5));
  CHECK(filter.update(10.0) == doctest::Approx(8.75));
  CHECK(filter.value() == doctest::Approx(8.75));
}

TEST_CASE("reset sets the stored value") {
  EmaFilter filter(0.5);
  filter.update(10.0);
  filter.reset();
  CHECK(filter.value() == doctest::Approx(0.0));
  filter.reset(4.0);
  CHECK(filter.update(4.0) == doctest::Approx(4.0));
}
