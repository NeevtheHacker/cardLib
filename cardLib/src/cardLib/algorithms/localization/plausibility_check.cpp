// plausibility_check.cpp
// Why: implements the SPEC section 4 plausibility check.
// Owns: nothing beyond PlausibilityCheck's own state.
#include "cardLib/algorithms/localization/plausibility_check.hpp"

#include <cmath>

#include "cardLib/algorithms/math/loop_timing.hpp"

namespace cardLib {

namespace {
constexpr std::uint32_t REJECTIONS_TO_FAULT = 3;
}

double PlausibilityCheck::update(std::optional<double> totalInches, double dtSeconds) {
  m_lastChangeMeasured = false;
  if (!totalInches.has_value()) {
    return reject();
  }
  if (!m_hasBaseline) {
    m_lastInches = *totalInches;
    m_hasBaseline = true;
    return 0.0;
  }

  const double dt = dtSeconds > 0.0 ? dtSeconds : LOOP_PERIOD_SECONDS;
  const double change = *totalInches - m_lastInches;
  if (std::abs(change) > m_margin * m_maxSpeedInchesPerSecond * dt) {
    return reject();
  }

  m_lastInches = *totalInches;
  m_flagged = false;
  m_rejectionsInARow = 0;
  m_lastChangeMeasured = true;
  return change;
}

void PlausibilityCheck::rebaseline(std::optional<double> totalInches) {
  m_hasBaseline = totalInches.has_value();
  m_lastInches = totalInches.value_or(0.0);
  m_flagged = false;
  m_faulted = false;
  m_rejectionsInARow = 0;
}

double PlausibilityCheck::reject() {
  m_flagged = true;
  m_hasBaseline = false;
  ++m_rejectionsInARow;
  if (m_rejectionsInARow >= REJECTIONS_TO_FAULT) {
    m_faulted = true;
  }
  return 0.0;
}

}  // namespace cardLib
