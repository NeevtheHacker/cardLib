// field_centric_test.cpp
// Why: proves field-centric sticks: stick forward always moves the robot away from the driver,
//      whichever way it faces (SPEC 20.11).
// Owns: tests for algorithms/control/field_centric.
#include "cardLib/algorithms/control/field_centric.hpp"

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
