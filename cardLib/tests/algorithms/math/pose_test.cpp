// pose_test.cpp
// Why: proves the robot-local <-> field transforms match SPEC section 1 at every quadrant.
// Owns: tests for algorithms/math/pose.
#include "cardLib/algorithms/math/pose.hpp"

#include <initializer_list>

#include "doctest.h"

using namespace cardLib;

TEST_CASE("fieldToLocal: facing +x, a point 10 in along +x is 10 in ahead (the SPEC check)") {
  const LocalOffset local = fieldToLocal({0.0, 0.0, 90.0}, {10.0, 0.0});
  CHECK(local.forwardInches == doctest::Approx(10.0));
  CHECK(local.rightInches == doctest::Approx(0.0).epsilon(1e-12));
}

TEST_CASE("fieldToLocal at 0, 180 and 270 degrees") {
  CHECK(fieldToLocal({0.0, 0.0, 0.0}, {0.0, 10.0}).forwardInches == doctest::Approx(10.0));
  CHECK(fieldToLocal({0.0, 0.0, 0.0}, {10.0, 0.0}).rightInches == doctest::Approx(10.0));
  CHECK(fieldToLocal({0.0, 0.0, 180.0}, {0.0, -10.0}).forwardInches == doctest::Approx(10.0));
  CHECK(fieldToLocal({0.0, 0.0, 270.0}, {-10.0, 0.0}).forwardInches == doctest::Approx(10.0));
  CHECK(fieldToLocal({0.0, 0.0, 90.0}, {0.0, -10.0}).rightInches == doctest::Approx(10.0));
}

TEST_CASE("fieldToLocal is measured from the robot's position") {
  const LocalOffset local = fieldToLocal({5.0, 5.0, 0.0}, {8.0, 15.0});
  CHECK(local.forwardInches == doctest::Approx(10.0));
  CHECK(local.rightInches == doctest::Approx(3.0));
}

TEST_CASE("localToField: facing +x, a point 10 in ahead is 10 in along +x") {
  const Point field = localToField({1.0, 2.0, 90.0}, {0.0, 10.0});
  CHECK(field.xInches == doctest::Approx(11.0));
  CHECK(field.yInches == doctest::Approx(2.0));
}

TEST_CASE("localToField then fieldToLocal round-trips at 0, 90, 180, 270 and 37 degrees") {
  for (const double headingDegrees : {0.0, 90.0, 180.0, 270.0, 37.0}) {
    CAPTURE(headingDegrees);
    const Pose pose{4.0, -7.0, headingDegrees};
    const LocalOffset start{3.0, -5.0};
    const LocalOffset back = fieldToLocal(pose, localToField(pose, start));
    CHECK(back.rightInches == doctest::Approx(start.rightInches));
    CHECK(back.forwardInches == doctest::Approx(start.forwardInches));
  }
}
