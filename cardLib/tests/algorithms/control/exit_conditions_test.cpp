// exit_conditions_test.cpp
// Why: proves each SPEC section 9 result fires at the right time, in the right order.
// Owns: tests for algorithms/control/exit_conditions, driven by a fake 10 ms clock.
#include "cardLib/algorithms/control/exit_conditions.hpp"

#include <array>
#include <cstdint>
#include <optional>

#include "doctest.h"

using namespace cardLib;

namespace {

constexpr ExitSettings MOVE_SETTINGS{.windows = MOVE_EXIT_WINDOWS, .timeoutMs = 3000};

// Runs the checker every 10 ms with the same inputs until it fires or maxMs passes.
std::optional<std::uint32_t> firstFireMs(ExitConditions& exits, ExitInputs inputs,
                                         std::uint32_t maxMs, ExitDecision* decision = nullptr) {
  for (std::uint32_t timeMs = 0; timeMs <= maxMs; timeMs += 10) {
    inputs.elapsedMs = timeMs;
    if (const auto result = exits.check(inputs)) {
      if (decision != nullptr) {
        *decision = *result;
      }
      return timeMs;
    }
  }
  return std::nullopt;
}

}  // namespace

TEST_CASE("not calibrated returns NotCalibrated immediately") {
  ExitConditions exits(MOVE_SETTINGS);
  const auto decision = exits.check({.error = 10.0, .calibrated = false});
  REQUIRE(decision.has_value());
  CHECK(decision->result == MotionResult::NotCalibrated);
  CHECK(decision->brake);
}

TEST_CASE("a sensor fault returns SensorFault") {
  ExitConditions exits(MOVE_SETTINGS);
  const auto decision = exits.check({.error = 10.0, .sensorFault = true});
  REQUIRE(decision.has_value());
  CHECK(decision->result == MotionResult::SensorFault);
}

TEST_CASE("cancel returns Cancelled") {
  ExitConditions exits(MOVE_SETTINGS);
  const auto decision = exits.check({.error = 10.0, .cancelled = true});
  REQUIRE(decision.has_value());
  CHECK(decision->result == MotionResult::Cancelled);
}

TEST_CASE("earlyExit settles at once, without braking") {
  ExitConditions exits({.windows = MOVE_EXIT_WINDOWS, .earlyExit = 2.0, .timeoutMs = 3000});
  CHECK_FALSE(exits.check({.error = 2.5, .velocity = 30.0}).has_value());
  const auto decision = exits.check({.error = 1.9, .velocity = 30.0, .elapsedMs = 10});
  REQUIRE(decision.has_value());
  CHECK(decision->result == MotionResult::Settled);
  CHECK_FALSE(decision->brake);
}

TEST_CASE("the small window fires after exactly 100 ms, not 90") {
  ExitConditions exits(MOVE_SETTINGS);
  ExitDecision decision{};
  CHECK(firstFireMs(exits, {.error = 0.5, .velocity = 1.0}, 1000, &decision) == 100u);
  CHECK(decision.result == MotionResult::Settled);
  CHECK(decision.brake);
}

TEST_CASE("small error but too fast never fires the small window; the large one fires at 500") {
  ExitConditions exits(MOVE_SETTINGS);
  CHECK(firstFireMs(exits, {.error = 0.5, .velocity = 5.0}, 1000) == 500u);
}

TEST_CASE("the large window fires after 500 ms") {
  ExitConditions exits(MOVE_SETTINGS);
  CHECK(firstFireMs(exits, {.error = 2.5, .velocity = 10.0}, 1000) == 500u);
}

TEST_CASE("leaving a window resets its timer") {
  ExitConditions exits(MOVE_SETTINGS);
  for (std::uint32_t timeMs = 0; timeMs <= 400; timeMs += 10) {
    CHECK_FALSE(exits.check({.error = 2.5, .velocity = 10.0, .elapsedMs = timeMs}).has_value());
  }
  CHECK_FALSE(exits.check({.error = 5.0, .velocity = 10.0, .elapsedMs = 410}).has_value());
  for (std::uint32_t timeMs = 420; timeMs < 920; timeMs += 10) {
    CHECK_FALSE(exits.check({.error = 2.5, .velocity = 10.0, .elapsedMs = timeMs}).has_value());
  }
  CHECK(exits.check({.error = 2.5, .velocity = 10.0, .elapsedMs = 920}).has_value());
}

TEST_CASE("a stall far from the target returns Stalled after 300 ms") {
  ExitConditions exits(MOVE_SETTINGS);
  ExitDecision decision{};
  CHECK(firstFireMs(exits, {.error = 10.0, .velocity = 0.1}, 1000, &decision) == 300u);
  CHECK(decision.result == MotionResult::Stalled);
}

TEST_CASE("with current stall detection on, a stall also needs the current check") {
  ExitConditions exits(
      {.windows = MOVE_EXIT_WINDOWS, .timeoutMs = 3000, .useCurrentForStall = true});
  CHECK_FALSE(firstFireMs(exits, {.error = 10.0, .velocity = 0.1}, 1000).has_value());
  exits.reset();
  CHECK(firstFireMs(exits, {.error = 10.0, .velocity = 0.1, .currentConfirmsStall = true}, 1000) ==
        300u);
}

TEST_CASE("currentConfirmsStall needs at least half the motors over the threshold") {
  const std::array<double, 4> twoHigh{2.5, 2.4, 0.5, 0.4};
  const std::array<double, 4> oneHigh{2.5, 0.5, 0.5, 0.4};
  CHECK(currentConfirmsStall(twoHigh, 2.0, 60.0, 20.0));
  CHECK_FALSE(currentConfirmsStall(oneHigh, 2.0, 60.0, 20.0));
  CHECK_FALSE(currentConfirmsStall(twoHigh, 2.0, 10.0, 20.0));
  CHECK(currentConfirmsStall(twoHigh, 2.0, -60.0, 20.0));
}

TEST_CASE("timeout fires once elapsed passes timeoutMs") {
  ExitConditions exits(MOVE_SETTINGS);
  ExitDecision decision{};
  CHECK(firstFireMs(exits, {.error = 10.0, .velocity = 20.0}, 4000, &decision) == 3010u);
  CHECK(decision.result == MotionResult::TimedOut);
}

TEST_CASE("results are checked in SPEC order: cancel beats timeout, fault beats cancel") {
  ExitConditions exits(MOVE_SETTINGS);
  auto decision = exits.check({.error = 10.0, .elapsedMs = 5000, .cancelled = true});
  REQUIRE(decision.has_value());
  CHECK(decision->result == MotionResult::Cancelled);
  decision = exits.check({.error = 10.0, .sensorFault = true, .cancelled = true});
  REQUIRE(decision.has_value());
  CHECK(decision->result == MotionResult::SensorFault);
}
