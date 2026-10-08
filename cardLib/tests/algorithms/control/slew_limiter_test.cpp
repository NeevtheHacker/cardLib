// slew_limiter_test.cpp
// Why: proves the two-rate slew (SPEC section 7) limits speeding up and slowing down correctly.
// Owns: tests for algorithms/control/slew_limiter.
#include "cardLib/algorithms/control/slew_limiter.hpp"

#include "doctest.h"

using namespace cardLib;

namespace {
constexpr double DT = 0.01;
constexpr SlewRates RATES{.accelPerSecond = 500.0, .decelPerSecond = 1500.0};
}  // namespace

TEST_CASE("speeding up is limited to accel * dt per loop") {
  SlewLimiter slew(RATES);
  CHECK(slew.update(127.0, DT) == doctest::Approx(5.0));
  CHECK(slew.update(127.0, DT) == doctest::Approx(10.0));
}

TEST_CASE("slowing down is limited to decel * dt per loop") {
  SlewLimiter slew(RATES);
  slew.reset(100.0);
  CHECK(slew.update(0.0, DT) == doctest::Approx(85.0));
}

TEST_CASE("slowing down by less than one step passes exactly") {
  SlewLimiter slew(RATES);
  slew.reset(10.0);
  CHECK(slew.update(5.0, DT) == doctest::Approx(5.0));
}

TEST_CASE("a limit of 0 is off") {
  SlewLimiter slew({.accelPerSecond = 0.0, .decelPerSecond = 0.0});
  CHECK(slew.update(127.0, DT) == doctest::Approx(127.0));
  CHECK(slew.update(20.0, DT) == doctest::Approx(20.0));
}

TEST_CASE("a sign flip counts as slowing down") {
  SlewLimiter slew(RATES);
  slew.reset(50.0);
  CHECK(slew.update(-127.0, DT) == doctest::Approx(35.0));
}

TEST_CASE("a sign flip stops at zero for one loop, then speeds up at the accel rate") {
  SlewLimiter slew(RATES);
  slew.reset(5.0);
  CHECK(slew.update(-100.0, DT) == doctest::Approx(0.0));
  CHECK(slew.update(-100.0, DT) == doctest::Approx(-5.0));

  SlewLimiter noDecel({.accelPerSecond = 500.0, .decelPerSecond = 0.0});
  noDecel.reset(-80.0);
  CHECK(noDecel.update(60.0, DT) == doctest::Approx(0.0));
  CHECK(noDecel.update(60.0, DT) == doctest::Approx(5.0));
}

TEST_CASE("negative outputs mirror positive ones") {
  SlewLimiter slew(RATES);
  CHECK(slew.update(-127.0, DT) == doctest::Approx(-5.0));
  slew.reset(-100.0);
  CHECK(slew.update(-50.0, DT) == doctest::Approx(-85.0));
}

TEST_CASE("reset seeds the output, for chaining motions") {
  SlewLimiter slew(RATES);
  slew.reset(60.0);
  CHECK(slew.output() == doctest::Approx(60.0));
  CHECK(slew.update(127.0, DT) == doctest::Approx(65.0));
}
