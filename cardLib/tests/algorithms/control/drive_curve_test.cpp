// drive_curve_test.cpp
// Why: proves the driver deadzone and expo curve (SPEC section 10).
// Owns: tests for algorithms/control/drive_curve.
#include "cardLib/algorithms/control/drive_curve.hpp"

#include <cmath>
#include <initializer_list>

#include "doctest.h"

using namespace cardLib;

TEST_CASE("inputs under the deadzone become 0") {
  const DriveCurve curve{.deadzone = 3.0, .curveGain = 0.0};
  CHECK(applyDriveCurve(2.0, curve) == doctest::Approx(0.0));
  CHECK(applyDriveCurve(-2.0, curve) == doctest::Approx(0.0));
  CHECK(applyDriveCurve(3.0, curve) == doctest::Approx(3.0));
}

TEST_CASE("curve gain 0 is linear") {
  const DriveCurve curve{.deadzone = 0.0, .curveGain = 0.0};
  for (const double input : {-127.0, -64.0, 0.0, 1.0, 50.0, 127.0}) {
    CHECK(applyDriveCurve(input, curve) == doctest::Approx(input));
  }
}

TEST_CASE("full stick is always full speed, whatever the gain") {
  for (const double gain : {0.0, 2.0, 5.0, 10.0, 30.0}) {
    const DriveCurve curve{.deadzone = 3.0, .curveGain = gain};
    CHECK(applyDriveCurve(127.0, curve) == doctest::Approx(127.0));
    CHECK(applyDriveCurve(-127.0, curve) == doctest::Approx(-127.0));
  }
}

TEST_CASE("a positive gain is gentler near center, odd-symmetric and monotonic") {
  const DriveCurve curve{.deadzone = 0.0, .curveGain = 5.0};
  CHECK(std::abs(applyDriveCurve(50.0, curve)) < 50.0);
  CHECK(applyDriveCurve(-50.0, curve) == doctest::Approx(-applyDriveCurve(50.0, curve)));
  double previous = applyDriveCurve(0.0, curve);
  for (int input = 1; input <= 127; ++input) {
    const double output = applyDriveCurve(input, curve);
    CHECK(output > previous);
    previous = output;
  }
}

TEST_CASE("inputs beyond full stick are clamped") {
  const DriveCurve curve{.deadzone = 3.0, .curveGain = 5.0};
  CHECK(applyDriveCurve(200.0, curve) == doctest::Approx(127.0));
}
