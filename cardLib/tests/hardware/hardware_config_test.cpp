// hardware_config_test.cpp
// Why: proves the wheel sizes, gearing and motor rpm that every drive's top speed is built from
//      (SPEC 20.1, 20.2, 20.13).
// Owns: tests for hardware/hardware_config and hardware/required_value.
#include "cardLib/hardware/hardware_config.hpp"

#include "cardLib/hardware/required_value.hpp"
#include "doctest.h"

using namespace cardLib;

static_assert(Gearing(36, 48).ratio() == 0.75);
static_assert(Gearing(0.75).ratio() == 0.75);
static_assert(wheelRpm(600.0, {36, 48}) == 450.0);
static_assert(motorRpm(MotorType::Exp5_5W, Cartridge::Blue600) == 200.0);
static_assert(effectiveCartridge({.cartridge = Cartridge::Red100, .type = MotorType::Exp5_5W}) ==
              Cartridge::Green200);
static_assert(RequiredValue<double>(3.231).value() == 3.231);

TEST_CASE("omni wheel diameters are the LemLib measured values") {
  CHECK(Omniwheel::NEW_2 == 2.125);
  CHECK(Omniwheel::NEW_275 == 2.75);
  CHECK(Omniwheel::OLD_275 == 2.75);
  CHECK(Omniwheel::NEW_275_HALF == 2.744);
  CHECK(Omniwheel::OLD_275_HALF == 2.74);
  CHECK(Omniwheel::NEW_325 == 3.25);
  CHECK(Omniwheel::OLD_325 == 3.25);
  CHECK(Omniwheel::NEW_325_HALF == 3.246);
  CHECK(Omniwheel::OLD_325_HALF == 3.246);
  CHECK(Omniwheel::NEW_4 == 4.0);
  CHECK(Omniwheel::OLD_4 == 4.18);
  CHECK(Omniwheel::NEW_4_HALF == 3.995);
  CHECK(Omniwheel::OLD_4_HALF == 4.175);
}

TEST_CASE("an 11 W motor runs at its cartridge's rpm; a 5.5 W motor always at 200") {
  CHECK(motorRpm({.cartridge = Cartridge::Red100}) == 100.0);
  CHECK(motorRpm({.cartridge = Cartridge::Green200}) == 200.0);
  CHECK(motorRpm({.cartridge = Cartridge::Blue600}) == 600.0);
  CHECK(motorRpm({.cartridge = Cartridge::Blue600, .type = MotorType::Exp5_5W}) == 200.0);
}

TEST_CASE("required values read like the plain number") {
  const RequiredValue<double> diameter = Omniwheel::NEW_325;
  const double asDouble = diameter;
  CHECK(asDouble == 3.25);
}
