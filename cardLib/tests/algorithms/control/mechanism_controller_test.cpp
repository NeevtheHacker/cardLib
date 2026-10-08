// mechanism_controller_test.cpp
// Why: proves the mechanism PID adds gravity hold, clamps targets and respects hard stops (SPEC 7).
// Owns: tests for algorithms/control/mechanism_controller.
#include "cardLib/algorithms/control/mechanism_controller.hpp"

#include "doctest.h"

using namespace cardLib;

namespace {
constexpr double DT = 0.01;
}

TEST_CASE("an arm adds kG * cos(angle from horizontal)") {
  MechanismController arm({.gravity = GravityMode::Arm, .gravityGain = 10.0});
  CHECK(arm.update(0.0, 0.0, DT) == doctest::Approx(10.0));
  CHECK(arm.update(90.0, 90.0, DT) == doctest::Approx(0.0).epsilon(1e-9));
  CHECK(arm.update(180.0, 180.0, DT) == doctest::Approx(-10.0));
  CHECK(arm.update(60.0, 60.0, DT) == doctest::Approx(5.0));
}

TEST_CASE("a lift adds a constant kG") {
  MechanismController lift({.gravity = GravityMode::Lift, .gravityGain = 8.0});
  CHECK(lift.update(3.0, 3.0, DT) == doctest::Approx(8.0));
  CHECK(lift.update(20.0, 20.0, DT) == doctest::Approx(8.0));
}

TEST_CASE("no gravity mode is a plain PID") {
  MechanismController plain({.gains = {.kP = 2.0}});
  CHECK(plain.update(10.0, 4.0, DT) == doctest::Approx(12.0));
}

TEST_CASE("soft limits clamp the target") {
  MechanismController arm({.gains = {.kP = 1.0}, .minTarget = 0.0, .maxTarget = 100.0});
  CHECK(arm.update(200.0, 90.0, DT) == doctest::Approx(10.0));
  CHECK(arm.update(-50.0, 5.0, DT) == doctest::Approx(-5.0));
  CHECK(arm.clampTarget(150.0) == doctest::Approx(100.0));
}

TEST_CASE("output pushing into a hard stop is cut inside the band; moving away is allowed") {
  const MechanismConfig config{.gains = {.kP = 1.0},
                               .hardStopMin = 0.0,
                               .hardStopMax = 100.0,
                               .hardStopBand = 1.0};
  MechanismController arm(config);
  CHECK(arm.update(150.0, 99.5, DT) == doctest::Approx(0.0));
  CHECK(arm.update(50.0, 99.5, DT) == doctest::Approx(-49.5));
  CHECK(arm.update(-20.0, 0.5, DT) == doctest::Approx(0.0));
  CHECK(arm.update(20.0, 0.5, DT) == doctest::Approx(19.5));
  CHECK(arm.update(150.0, 98.0, DT) == doctest::Approx(52.0));
}

TEST_CASE("output is clamped to the speed scale") {
  MechanismController arm({.gains = {.kP = 100.0}});
  CHECK(arm.update(10.0, 0.0, DT) == doctest::Approx(127.0));
  CHECK(arm.update(-10.0, 0.0, DT) == doctest::Approx(-127.0));
}
