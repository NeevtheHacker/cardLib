// heading_disagreement.cpp
// Why: implements the SPEC section 4 disagreement check.
// Owns: nothing beyond HeadingDisagreement's window.
#include "cardLib/algorithms/localization/heading_disagreement.hpp"

#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

bool HeadingDisagreement::update(double wheelChangeDegrees, double imuChangeDegrees) {
  m_differences[m_next] = wrapDegrees(wheelChangeDegrees - imuChangeDegrees);
  m_next = (m_next + 1) % WINDOW_LOOPS;

  double sum = 0.0;
  for (const double difference : m_differences) {
    sum += difference;
  }
  m_suspect = std::abs(sum) > m_thresholdDegrees;
  return m_suspect;
}

void HeadingDisagreement::reset() {
  m_differences.fill(0.0);
  m_next = 0;
  m_suspect = false;
}

}  // namespace cardLib
