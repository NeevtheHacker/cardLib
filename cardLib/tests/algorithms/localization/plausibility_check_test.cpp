// plausibility_check_test.cpp
// Why: proves bad distance readings count as zero motion and never invent movement (SPEC 4).
// Owns: tests for algorithms/localization/plausibility_check.
#include "cardLib/algorithms/localization/plausibility_check.hpp"

#include <optional>

#include "doctest.h"

using namespace cardLib;

namespace {
constexpr double DT = 0.01;
// 60 in/s max: the limit per loop is 1.5 * 60 * 0.01 = 0.9 in.
constexpr double MAX_SPEED = 60.0;
}  // namespace

TEST_CASE("the first reading only sets the baseline") {
  PlausibilityCheck check(MAX_SPEED);
  CHECK(check.update(100.0, DT) == doctest::Approx(0.0));
  CHECK(check.update(100.5, DT) == doctest::Approx(0.5));
  CHECK_FALSE(check.flagged());
}

TEST_CASE("a change within 1.5 * v_max * dt passes") {
  PlausibilityCheck check(MAX_SPEED);
  check.update(0.0, DT);
  CHECK(check.update(0.89, DT) == doctest::Approx(0.89));
  CHECK(check.update(0.0, DT) == doctest::Approx(-0.89));
}

TEST_CASE("a jump over the limit counts as zero and flags the sensor") {
  PlausibilityCheck check(MAX_SPEED);
  check.update(0.0, DT);
  CHECK(check.update(5.0, DT) == doctest::Approx(0.0));
  CHECK(check.flagged());
  CHECK_FALSE(check.faulted());
}

TEST_CASE("the first good reading after a rejection only re-baselines, then normal again") {
  PlausibilityCheck check(MAX_SPEED);
  check.update(0.0, DT);
  check.update(50.0, DT);
  CHECK(check.update(50.3, DT) == doctest::Approx(0.0));
  CHECK(check.update(50.8, DT) == doctest::Approx(0.5));
  CHECK_FALSE(check.flagged());
}

TEST_CASE("a failed read (empty) counts as zero and needs a re-baseline") {
  PlausibilityCheck check(MAX_SPEED);
  check.update(0.0, DT);
  check.update(0.5, DT);
  CHECK(check.update(std::nullopt, DT) == doctest::Approx(0.0));
  CHECK(check.flagged());
  CHECK(check.update(1.2, DT) == doctest::Approx(0.0));
  CHECK(check.update(1.6, DT) == doctest::Approx(0.4));
}

TEST_CASE("three rejections in a row mark the sensor faulted, and it stays faulted") {
  PlausibilityCheck check(MAX_SPEED);
  check.update(0.0, DT);
  check.update(std::nullopt, DT);
  check.update(std::nullopt, DT);
  CHECK_FALSE(check.faulted());
  check.update(std::nullopt, DT);
  CHECK(check.faulted());
  check.update(1.0, DT);
  check.update(1.1, DT);
  CHECK(check.faulted());
  check.rebaseline(1.1);
  CHECK_FALSE(check.faulted());
}

TEST_CASE("good readings between rejections reset the count") {
  PlausibilityCheck check(MAX_SPEED);
  check.update(0.0, DT);
  check.update(std::nullopt, DT);
  check.update(std::nullopt, DT);
  check.update(0.0, DT);
  check.update(0.1, DT);
  check.update(std::nullopt, DT);
  CHECK_FALSE(check.faulted());
}

TEST_CASE("a rejected loop never repeats the previous delta") {
  PlausibilityCheck check(MAX_SPEED);
  check.update(0.0, DT);
  CHECK(check.update(0.8, DT) == doctest::Approx(0.8));
  CHECK(check.update(9.0, DT) == doctest::Approx(0.0));
}

TEST_CASE("rebaseline sets a fresh zero, as setPose does") {
  PlausibilityCheck check(MAX_SPEED);
  check.update(0.0, DT);
  check.rebaseline(300.0);
  CHECK(check.update(300.5, DT) == doctest::Approx(0.5));
}

TEST_CASE("a longer dt allows a proportionally larger change") {
  PlausibilityCheck check(MAX_SPEED);
  check.update(0.0, DT);
  CHECK(check.update(1.7, 0.02) == doctest::Approx(1.7));
}

TEST_CASE("lastChangeMeasured is true only for an accepted change") {
  PlausibilityCheck check(MAX_SPEED);
  check.update(0.0, DT);
  CHECK_FALSE(check.lastChangeMeasured());
  check.update(0.5, DT);
  CHECK(check.lastChangeMeasured());
  check.update(9.0, DT);
  CHECK_FALSE(check.lastChangeMeasured());
  check.update(9.1, DT);
  CHECK_FALSE(check.lastChangeMeasured());
  check.update(std::nullopt, DT);
  CHECK_FALSE(check.lastChangeMeasured());
}

TEST_CASE("measure: one rejected reading is a zero delta from a sensor that is still usable") {
  PlausibilityCheck check(MAX_SPEED);
  check.measure(0.0, DT);
  const WheelDelta rejected = check.measure(9.0, DT);
  CHECK(rejected.inches == doctest::Approx(0.0));
  CHECK(rejected.usable);
  const WheelDelta failed = check.measure(std::nullopt, DT);
  CHECK(failed.inches == doctest::Approx(0.0));
  CHECK(failed.usable);
}

TEST_CASE("measure: the re-baseline loop after a reject is usable too, then normal") {
  PlausibilityCheck check(MAX_SPEED);
  check.measure(0.0, DT);
  check.measure(9.0, DT);
  const WheelDelta rebaseline = check.measure(9.2, DT);
  CHECK(rebaseline.inches == doctest::Approx(0.0));
  CHECK(rebaseline.usable);
  const WheelDelta normal = check.measure(9.6, DT);
  CHECK(normal.inches == doctest::Approx(0.4));
  CHECK(normal.usable);
}

TEST_CASE("measure: three rejections in a row make the sensor unusable until rebaseline") {
  PlausibilityCheck check(MAX_SPEED);
  check.measure(0.0, DT);
  CHECK(check.measure(std::nullopt, DT).usable);
  CHECK(check.measure(std::nullopt, DT).usable);
  CHECK_FALSE(check.measure(std::nullopt, DT).usable);
  CHECK_FALSE(check.measure(0.0, DT).usable);
  CHECK_FALSE(check.measure(0.1, DT).usable);
  check.rebaseline(0.1);
  CHECK(check.measure(0.2, DT).usable);
}

TEST_CASE("every rejection is counted, for the log") {
  PlausibilityCheck check(MAX_SPEED);
  check.measure(0.0, DT);
  check.measure(std::nullopt, DT);
  check.measure(0.0, DT);
  check.measure(5.0, DT);
  CHECK(check.totalRejections() == 2u);
}
