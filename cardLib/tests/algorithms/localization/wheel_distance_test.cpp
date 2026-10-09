// wheel_distance_test.cpp
// Why: proves raw sensor units become inches exactly as SPEC section 5 step 1 says.
// Owns: tests for algorithms/localization/wheel_distance.
#include "cardLib/algorithms/localization/wheel_distance.hpp"

#include <numbers>

#include "doctest.h"

using namespace cardLib;

TEST_CASE("one rotation-sensor turn (36000 cd) on a 2.75 in wheel is 2.75 pi inches") {
  CHECK(rotationSensorInches(36000.0, 2.75, 1.0) == doctest::Approx(2.75 * std::numbers::pi));
  CHECK(rotationSensorInches(-18000.0, 2.75, 1.0) == doctest::Approx(-1.375 * std::numbers::pi));
  CHECK(rotationSensorInches(36000.0, 2.75, 0.5) == doctest::Approx(1.375 * std::numbers::pi));
}

TEST_CASE("one motor output turn (360 deg) through a 36:48 gear on a 3.25 in wheel") {
  CHECK(motorEncoderInches(360.0, 3.25, 0.75, 1.0) ==
        doctest::Approx(0.75 * 3.25 * std::numbers::pi));
  CHECK(motorEncoderInches(360.0, 3.25, 0.75, 0.987) ==
        doctest::Approx(0.987 * 0.75 * 3.25 * std::numbers::pi));
}

TEST_CASE("one ADI encoder turn (360 ticks) on a 2.75 in wheel is 2.75 pi inches") {
  CHECK(adiEncoderInches(360.0, 2.75, 1.0) == doctest::Approx(2.75 * std::numbers::pi));
}

TEST_CASE("max wheel speed is pi * D * rpm / 60") {
  CHECK(maxWheelSpeedInchesPerSecond(3.25, 450.0) ==
        doctest::Approx(std::numbers::pi * 3.25 * 450.0 / 60.0));
}

TEST_CASE("an Encoder's degrees on a 2.75 in wheel: 360 degrees is 2.75 pi inches") {
  CHECK(encoderDegreesToInches(360.0, 2.75, 1.0) == doctest::Approx(2.75 * std::numbers::pi));
  CHECK(encoderDegreesToInches(-90.0, 2.0, 2.0) == doctest::Approx(-std::numbers::pi));
}
