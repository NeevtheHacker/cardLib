// drive_slip_check.cpp
// Why: implements the SPEC 20.12 drive slip flag.
// Owns: nothing beyond DriveSlipCheck's window.
#include "cardLib/algorithms/localization/drive_slip_check.hpp"

#include <cmath>

namespace cardLib {

bool DriveSlipCheck::update(double residualInches) {
  m_residuals[m_next] = std::abs(residualInches);
  m_next = (m_next + 1) % WINDOW_LOOPS;

  double sum = 0.0;
  for (const double residual : m_residuals) {
    sum += residual;
  }
  m_suspect = sum > m_thresholdInches;
  return m_suspect;
}

void DriveSlipCheck::reset() {
  m_residuals.fill(0.0);
  m_next = 0;
  m_suspect = false;
}

}  // namespace cardLib
