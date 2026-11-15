// field_centric_test.cpp
// Why: proves field-centric sticks: stick forward always moves the robot away from the driver,
//      whichever way it faces (SPEC 20.11).
// Owns: tests for algorithms/control/field_centric.
#include "cardLib/algorithms/control/field_centric.hpp"

#include <cmath>

#include "cardLib/algorithms/control/drive_curve.hpp"
#include "cardLib/algorithms/math/pose.hpp"
#include "doctest.h"

using namespace cardLib;

TEST_CASE("facing field forward, field-centric sticks are the robot sticks") {
  const StickVector sticks = fieldCentricSticks(80.0, -30.0, 0.0);
  CHECK(sticks.forward == doctest::Approx(80.0));
  CHECK(sticks.strafe == doctest::Approx(-30.0));
}

TEST_CASE("facing right (90), stick forward becomes a strafe to the robot's left") {
  const StickVector sticks = fieldCentricSticks(100.0, 0.0, 90.0);
  CHECK(sticks.forward == doctest::Approx(0.0).epsilon(1e-9));
  CHECK(sticks.strafe == doctest::Approx(-100.0));
}

TEST_CASE("facing the driver (180), stick right becomes the robot's left") {
  const StickVector sticks = fieldCentricSticks(0.0, 50.0, 180.0);
  CHECK(sticks.strafe == doctest::Approx(-50.0));
}

TEST_CASE("the robot-frame command always points the robot along the field stick") {
  for (double heading = -180.0; heading < 180.0; heading += 30.0) {
    const StickVector sticks = fieldCentricSticks(60.0, 25.0, heading);
    // Turn the robot-frame command back into the field: it must be the stick vector.
    const Point field = localToField({0.0, 0.0, heading}, {sticks.strafe, sticks.forward});
    CHECK(field.xInches == doctest::Approx(25.0));
    CHECK(field.yInches == doctest::Approx(60.0));
  }
}

TEST_CASE("curved field-centric sticks feel the same whichever way the robot faces") {
  const DriveCurve curve{.deadzone = 3.0, .curveGain = 8.0};
  const double expected = applyDriveCurve(60.0, curve);
  for (double heading = -180.0; heading < 180.0; heading += 15.0) {
    const StickVector sticks = curvedFieldCentricSticks(60.0, 0.0, heading, curve);
    CHECK(std::hypot(sticks.forward, sticks.strafe) == doctest::Approx(expected));
    const Point field = localToField({0.0, 0.0, heading}, {sticks.strafe, sticks.forward});
    CHECK(field.xInches == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(field.yInches == doctest::Approx(expected));
  }
}

TEST_CASE("curved field-centric sticks curve the stick's length, not each axis") {
  const DriveCurve curve{.deadzone = 3.0, .curveGain = 8.0};
  const StickVector diagonal = curvedFieldCentricSticks(40.0, 30.0, 0.0, curve);
  CHECK(std::hypot(diagonal.forward, diagonal.strafe) ==
        doctest::Approx(applyDriveCurve(50.0, curve)));
  CHECK(diagonal.strafe / diagonal.forward == doctest::Approx(30.0 / 40.0));
}

TEST_CASE("curved field-centric sticks: radial deadzone and a full-stick limit of 127") {
  const DriveCurve curve{.deadzone = 3.0, .curveGain = 8.0};
  const StickVector small = curvedFieldCentricSticks(2.0, 2.0, 30.0, curve);
  CHECK(small.forward == 0.0);
  CHECK(small.strafe == 0.0);
  const StickVector corner = curvedFieldCentricSticks(127.0, 127.0, 30.0, curve);
  CHECK(std::hypot(corner.forward, corner.strafe) == doctest::Approx(127.0));
}
