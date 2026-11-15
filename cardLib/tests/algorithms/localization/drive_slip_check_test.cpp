// drive_slip_check_test.cpp
// Why: proves the forward-kinematics residual raises a drive slip flag over a 100 ms window
//      (SPEC 20.12), the same way heading disagreement does.
// Owns: tests for algorithms/localization/drive_slip_check.
#include "cardLib/algorithms/localization/drive_slip_check.hpp"

#include "doctest.h"

using namespace cardLib;

TEST_CASE("wheels that agree are never suspect") {
  DriveSlipCheck check;
  for (int i = 0; i < 50; ++i) {
    CHECK_FALSE(check.update(0.0));
  }
}

TEST_CASE("encoder noise below the threshold is not suspect") {
  DriveSlipCheck check;
  for (int i = 0; i < 50; ++i) {
    CHECK_FALSE(check.update(0.02));  // 0.2 in over the window, under 0.5
  }
}

TEST_CASE("0.1 in of residual per loop is suspect once it sums past 0.5 in") {
  DriveSlipCheck check;
  for (int i = 0; i < 5; ++i) {
    CHECK_FALSE(check.update(0.1));
  }
  CHECK(check.update(0.1));
  CHECK(check.suspect());
}

TEST_CASE("one bump is not suspect, and suspect clears once slip leaves the window") {
  DriveSlipCheck check;
  CHECK_FALSE(check.update(0.4));
  for (int i = 0; i < 6; ++i) {
    check.update(0.1);
  }
  CHECK(check.suspect());
  for (std::size_t i = 0; i < DriveSlipCheck::WINDOW_LOOPS; ++i) {
    check.update(0.0);
  }
  CHECK_FALSE(check.suspect());
}

TEST_CASE("reset clears the window") {
  DriveSlipCheck check;
  for (int i = 0; i < 10; ++i) {
    check.update(0.2);
  }
  REQUIRE(check.suspect());
  check.reset();
  CHECK_FALSE(check.suspect());
  CHECK_FALSE(check.update(0.1));
}

TEST_CASE("the threshold can be set") {
  DriveSlipCheck check(2.0);
  for (int i = 0; i < 10; ++i) {
    CHECK_FALSE(check.update(0.15));
  }
}
