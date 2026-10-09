// port_registry_test.cpp
// Why: proves the compile-time port checks catch duplicates and impossible ports (SPEC 17).
// Owns: tests for the constexpr part of hardware/port_registry (the header has no PROS in it).
#include "cardLib/hardware/port_registry.hpp"

#include <array>
#include <cstdint>

#include "doctest.h"

using namespace cardLib;

namespace {
constexpr std::array<std::uint8_t, 3> GOOD_SMART{1, 12, 21};
constexpr std::array<std::uint8_t, 3> DUPLICATE_SMART{4, 9, 4};
constexpr std::array<std::uint8_t, 2> OUT_OF_RANGE_SMART{0, 22};
constexpr std::array<std::uint8_t, 3> GOOD_ADI{'A', 'b', 3};
constexpr std::array<std::uint8_t, 2> DUPLICATE_ADI{'C', 3};
constexpr std::array<std::uint8_t, 1> BAD_ADI{'I'};
}  // namespace

static_assert(smartPortsValid(GOOD_SMART));
static_assert(adiPortsValid(GOOD_ADI));

TEST_CASE("Smart Ports must be 1 to 21 and unique") {
  CHECK(smartPortsValid(GOOD_SMART));
  CHECK_FALSE(smartPortsValid(DUPLICATE_SMART));
  CHECK_FALSE(smartPortsValid(OUT_OF_RANGE_SMART));
}

TEST_CASE("3-wire ports may be letters or numbers, and 'C' is the same port as 3") {
  CHECK(normalizeAdiPort('A') == 1);
  CHECK(normalizeAdiPort('h') == 8);
  CHECK(normalizeAdiPort(5) == 5);
  CHECK(normalizeAdiPort('I') == 0);
  CHECK(adiPortsValid(GOOD_ADI));
  CHECK_FALSE(adiPortsValid(DUPLICATE_ADI));
  CHECK_FALSE(adiPortsValid(BAD_ADI));
}
