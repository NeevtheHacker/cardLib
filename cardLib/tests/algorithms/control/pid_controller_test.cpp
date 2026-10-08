// pid_controller_test.cpp
// Why: proves the PID's guards (integral zone, sign flip, cap, no kick, overrun) per SPEC 7.
// Owns: tests for algorithms/control/pid_controller.
#include "cardLib/algorithms/control/pid_controller.hpp"

#include "doctest.h"

using namespace cardLib;

namespace {
constexpr double DT = 0.01;
}

TEST_CASE("P only: kP 10 and 2 in of error gives 10 * 2 = 20") {
  PidController pid({.kP = 10.0});
  CHECK(pid.update(2.0, DT) == doctest::Approx(20.0));
}

TEST_CASE("no D on the first update after construction or reset") {
  PidController pid({.kD = 1.0});
  CHECK(pid.update(10.0, DT) == doctest::Approx(0.0));
  CHECK(pid.update(8.0, DT) == doctest::Approx(-200.0));
  pid.reset();
  CHECK(pid.update(5.0, DT) == doctest::Approx(0.0));
}

TEST_CASE("integral only grows inside the integral zone") {
  PidController pid({.kI = 1.0, .integralZone = 5.0});
  CHECK(pid.update(10.0, DT) == doctest::Approx(0.0));
  CHECK(pid.update(3.0, DT) == doctest::Approx(0.03));
  CHECK(pid.update(3.0, DT) == doctest::Approx(0.06));
}

TEST_CASE("integral zone 0 means always integrate") {
  PidController pid({.kI = 1.0, .integralZone = 0.0});
  CHECK(pid.update(100.0, DT) == doctest::Approx(1.0));
}

TEST_CASE("sign flip resets the integral only when resetOnSignFlip is set") {
  PidController resetting({.kI = 1.0, .resetOnSignFlip = true});
  resetting.update(2.0, DT);
  resetting.update(2.0, DT);
  CHECK(resetting.update(-1.0, DT) == doctest::Approx(0.0));

  PidController keeping({.kI = 1.0, .resetOnSignFlip = false});
  keeping.update(2.0, DT);
  keeping.update(2.0, DT);
  CHECK(keeping.update(-1.0, DT) == doctest::Approx(0.03));
}

TEST_CASE("an error of exactly zero is not a sign flip") {
  PidController pid({.kI = 1.0, .resetOnSignFlip = true});
  pid.update(2.0, DT);
  pid.update(0.0, DT);
  CHECK(pid.update(2.0, DT) == doctest::Approx(0.04));
}

TEST_CASE("|kI * I| is capped at integralLimit") {
  PidController pid({.kI = 10.0, .integralLimit = 5.0});
  for (int i = 0; i < 1000; ++i) {
    pid.update(100.0, DT);
  }
  CHECK(pid.update(100.0, DT) == doctest::Approx(5.0));
  PidController negative({.kI = 10.0, .integralLimit = 5.0});
  for (int i = 0; i < 1000; ++i) {
    negative.update(-100.0, DT);
  }
  CHECK(negative.update(-100.0, DT) == doctest::Approx(-5.0));
}

TEST_CASE("kI = 0 is safe with a cap") {
  PidController pid({.kP = 1.0, .kI = 0.0, .integralLimit = 5.0});
  CHECK(pid.update(3.0, DT) == doctest::Approx(3.0));
}

TEST_CASE("angle mode wraps the change in error: 179 to -179 is a 2 degree step") {
  PidController pid({.kD = 1.0}, ErrorKind::Angle);
  pid.update(179.0, DT);
  CHECK(pid.update(-179.0, DT) == doctest::Approx(2.0 / DT));

  PidController distance({.kD = 1.0}, ErrorKind::Distance);
  distance.update(179.0, DT);
  CHECK(distance.update(-179.0, DT) == doctest::Approx(-358.0 / DT));
}

TEST_CASE("a loop overrun skips D and adds nothing to the integral") {
  PidController pid({.kI = 1.0, .kD = 1.0});
  pid.update(10.0, DT);
  CHECK_FALSE(pid.lastUpdateOverran());
  CHECK(pid.update(5.0, 0.03) == doctest::Approx(0.1));
  CHECK(pid.lastUpdateOverran());
  CHECK(pid.update(5.0, DT) == doctest::Approx(0.15));
  CHECK_FALSE(pid.lastUpdateOverran());
}

TEST_CASE("D smoothing: beta 1 is no filter, beta 0.5 halves a step") {
  PidController unfiltered({.kD = 1.0, .derivativeSmoothing = 1.0});
  unfiltered.update(0.0, DT);
  CHECK(unfiltered.update(1.0, DT) == doctest::Approx(100.0));

  PidController filtered({.kD = 1.0, .derivativeSmoothing = 0.5});
  filtered.update(0.0, DT);
  CHECK(filtered.update(1.0, DT) == doctest::Approx(50.0));
  CHECK(filtered.update(2.0, DT) == doctest::Approx(75.0));
}

TEST_CASE("setGains changes the gains without clearing state") {
  PidController pid({.kP = 1.0});
  pid.setGains({.kP = 2.0});
  CHECK(pid.gains().kP == doctest::Approx(2.0));
  CHECK(pid.update(3.0, DT) == doctest::Approx(6.0));
}
