// drive_mixer_test.cpp
// Why: proves mixing and desaturation keep the full turn command (SPEC section 7).
// Owns: tests for algorithms/control/drive_mixer.
#include "cardLib/algorithms/control/drive_mixer.hpp"

#include "doctest.h"

using namespace cardLib;

TEST_CASE("clockwise angular speeds up the left side") {
  const SideSpeeds sides = mixDrive(30.0, 20.0);
  CHECK(sides.left == doctest::Approx(50.0));
  CHECK(sides.right == doctest::Approx(10.0));
}

TEST_CASE("desaturation shrinks linear and keeps the full turn") {
  const SideSpeeds sides = mixDrive(100.0, 50.0);
  CHECK(sides.left == doctest::Approx(127.0));
  CHECK(sides.right == doctest::Approx(27.0));
  CHECK(sides.left - sides.right == doctest::Approx(100.0));
}

TEST_CASE("desaturation works backwards too") {
  const SideSpeeds sides = mixDrive(-100.0, 50.0);
  CHECK(sides.left == doctest::Approx(-27.0));
  CHECK(sides.right == doctest::Approx(-127.0));
}

TEST_CASE("angular is clamped to 127 first, leaving no room for linear") {
  const SideSpeeds sides = mixDrive(50.0, 200.0);
  CHECK(sides.left == doctest::Approx(127.0));
  CHECK(sides.right == doctest::Approx(-127.0));
}

TEST_CASE("speedToMillivolts scales 127 to 12000 and rounds") {
  CHECK(speedToMillivolts(127.0) == 12000);
  CHECK(speedToMillivolts(-127.0) == -12000);
  CHECK(speedToMillivolts(1.0) == 94);
  CHECK(speedToMillivolts(0.0) == 0);
  CHECK(speedToMillivolts(200.0) == 12000);
}
