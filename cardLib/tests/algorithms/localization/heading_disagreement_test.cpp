// heading_disagreement_test.cpp
// Why: proves wheel and IMU heading disagreement is flagged over a 100 ms window (SPEC 4).
// Owns: tests for algorithms/localization/heading_disagreement.
#include "cardLib/algorithms/localization/heading_disagreement.hpp"

#include "doctest.h"

using namespace cardLib;

TEST_CASE("agreeing sources are never suspect") {
  HeadingDisagreement check;
  for (int i = 0; i < 50; ++i) {
    CHECK_FALSE(check.update(1.0, 1.0));
  }
}

TEST_CASE("half a degree per loop of slip is suspect once it sums past 2 degrees") {
  HeadingDisagreement check;
  for (int i = 0; i < 4; ++i) {
    CHECK_FALSE(check.update(1.5, 1.0));
  }
  CHECK(check.update(1.5, 1.0));
  CHECK(check.suspect());
}

TEST_CASE("one 1 degree spike is not suspect") {
  HeadingDisagreement check;
  CHECK_FALSE(check.update(1.0, 0.0));
  for (int i = 0; i < 20; ++i) {
    CHECK_FALSE(check.update(0.0, 0.0));
  }
}

TEST_CASE("the difference is wrapped: 179 vs -179 is 2 degrees apart, not 358") {
  HeadingDisagreement check;
  CHECK_FALSE(check.update(179.0, -179.0));
}

TEST_CASE("suspect clears once the disagreement leaves the 10-loop window") {
  HeadingDisagreement check;
  for (int i = 0; i < 10; ++i) {
    check.update(0.5, 0.0);
  }
  CHECK(check.suspect());
  for (int i = 0; i < 5; ++i) {
    check.update(0.0, 0.0);
  }
  CHECK(check.suspect());
  for (int i = 0; i < 5; ++i) {
    check.update(0.0, 0.0);
  }
  CHECK_FALSE(check.suspect());
}

TEST_CASE("reset clears the window") {
  HeadingDisagreement check;
  for (int i = 0; i < 10; ++i) {
    check.update(0.5, 0.0);
  }
  check.reset();
  CHECK_FALSE(check.suspect());
  CHECK_FALSE(check.update(0.0, 0.0));
}
