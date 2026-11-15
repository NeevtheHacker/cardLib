// constexpr_math.hpp
// Why: sin, cos and abs that work at compile time on every compiler, so drive kinematics built
//      from a robot's build numbers can be constexpr (SPEC 20.7).
// Owns: constexprAbs, constexprSinDegrees, constexprCosDegrees. Degrees in; exact at multiples
//       of 90 degrees, so a tank's rows are exactly 0 and 1.
#pragma once

#include <numbers>

namespace cardLib {

constexpr double constexprAbs(double value) {
  return value < 0.0 ? -value : value;
}

namespace detail {

// Largest whole number not above value, for |value| well inside the range of long long.
constexpr double floorForAngles(double value) {
  const auto truncated = static_cast<double>(static_cast<long long>(value));
  return truncated > value ? truncated - 1.0 : truncated;
}

// Taylor series for sin(x) with |x| <= pi/2; 20 terms is far past double precision.
constexpr double sinSeries(double radians) {
  constexpr int TERMS = 20;
  const double squared = radians * radians;
  double term = radians;
  double sum = radians;
  for (int n = 1; n < TERMS; ++n) {
    term *= -squared / static_cast<double>((2 * n) * (2 * n + 1));
    sum += term;
  }
  return sum;
}

}  // namespace detail

// sin of an angle in degrees. Wraps to [-180, 180) first, folds into [-90, 90], then sums the
// series; 0, +-90 and -180 come out exact.
constexpr double constexprSinDegrees(double angleDegrees) {
  double wrapped = angleDegrees - 360.0 * detail::floorForAngles((angleDegrees + 180.0) / 360.0);
  if (wrapped == 0.0 || wrapped == -180.0) {
    return 0.0;
  }
  if (wrapped == 90.0) {
    return 1.0;
  }
  if (wrapped == -90.0) {
    return -1.0;
  }
  if (wrapped > 90.0) {
    wrapped = 180.0 - wrapped;
  } else if (wrapped < -90.0) {
    wrapped = -180.0 - wrapped;
  }
  return detail::sinSeries(wrapped * std::numbers::pi / 180.0);
}

// cos(a) = sin(a + 90), so cos 45 and sin 45 are the same number.
constexpr double constexprCosDegrees(double angleDegrees) {
  return constexprSinDegrees(angleDegrees + 90.0);
}

}  // namespace cardLib
