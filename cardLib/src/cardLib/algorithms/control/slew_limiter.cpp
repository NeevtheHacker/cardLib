// slew_limiter.cpp
// Why: implements the SPEC section 7 two-rate slew.
// Owns: nothing beyond SlewLimiter's own output.
#include "cardLib/algorithms/control/slew_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace cardLib {

double SlewLimiter::update(double target, double dtSeconds) {
  const bool crossesZero = target * m_output < 0.0;
  const bool speedingUp = !crossesZero && std::abs(target) > std::abs(m_output);
  const double rate = speedingUp ? m_rates.accelPerSecond : m_rates.decelPerSecond;

  double next = target;
  if (rate > 0.0) {
    const double maxStep = rate * dtSeconds;
    next = m_output + std::clamp(target - m_output, -maxStep, maxStep);
  }
  if (crossesZero && next * m_output < 0.0) {
    next = 0.0;
  }
  m_output = next;
  return m_output;
}

}  // namespace cardLib
