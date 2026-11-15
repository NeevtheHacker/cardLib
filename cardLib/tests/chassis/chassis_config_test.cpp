// chassis_config_test.cpp
// Why: proves the config values a drive can't work without have no default (SPEC 20.14), so
//      leaving one out is a compile error instead of one team's numbers.
// Owns: compile-time tests for chassis/chassis_config and the tracking wheel config.
#include "cardLib/chassis/chassis_config.hpp"

#include <type_traits>

#include "doctest.h"

using namespace cardLib;

static_assert(!std::is_default_constructible_v<Geometry>,
              "Geometry's wheel diameter and track width must be required");
static_assert(!std::is_default_constructible_v<TrackingWheelConfig>,
              "a tracking wheel's diameter must be required");
static_assert(std::is_default_constructible_v<ChassisConfig>,
              "a config built from presets needs no Geometry");

TEST_CASE("Geometry is only set for the left/right shorthand, and keeps what it is given") {
  const ChassisConfig presetConfig;
  CHECK_FALSE(presetConfig.geometry.has_value());

  ChassisConfig shorthand;
  shorthand.geometry = Geometry{.wheelDiameterInches = 3.25, .trackWidthInches = 12.5};
  REQUIRE(shorthand.geometry.has_value());
  CHECK(shorthand.geometry->wheelDiameterInches == 3.25);
  CHECK(shorthand.geometry->trackWidthInches == 12.5);
  CHECK(shorthand.geometry->externalRatio == 1.0);
}

TEST_CASE("a tracking wheel keeps its required diameter and neutral defaults") {
  const TrackingWheelConfig wheel{.diameterInches = Omniwheel::NEW_275};
  CHECK(wheel.diameterInches.value() == 2.75);
  CHECK(wheel.sensor == nullptr);
  CHECK(wheel.gearRatio == 1.0);
  CHECK(wheel.xInches == 0.0);
  CHECK(wheel.yInches == 0.0);
}
