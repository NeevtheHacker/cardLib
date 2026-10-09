// estimator_template_test.cpp
// Why: keeps the advanced/ template compiling and honoring the PoseEstimator contract, so the
//      file teams copy is never broken.
// Owns: tests for include/cardLib/advanced/_template/estimator_template.
#include "cardLib/advanced/_template/estimator_template.hpp"

#include "doctest.h"

using namespace cardLib;

TEST_CASE("the template is a PoseEstimator") {
  EstimatorTemplate estimator({});
  PoseEstimator& asInterface = estimator;
  asInterface.reset({1.0, 2.0, 30.0});
  const RobotState state = asInterface.update(SensorFrame{});
  CHECK(state.pose.xInches == doctest::Approx(1.0));
  CHECK(state.pose.yInches == doctest::Approx(2.0));
  CHECK(state.pose.headingDegrees == doctest::Approx(30.0));
}

TEST_CASE("reset replaces the pose") {
  EstimatorTemplate estimator({});
  estimator.reset({1.0, 2.0, 30.0});
  estimator.reset({-4.0, 5.0, -90.0});
  CHECK(estimator.update(SensorFrame{}).pose.xInches == doctest::Approx(-4.0));
}
