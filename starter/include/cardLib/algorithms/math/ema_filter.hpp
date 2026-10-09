// ema_filter.hpp
// Why: one-multiply smoothing for velocity estimates and the optional D filter (SPEC 4, 7).
// Owns: the filter's single stored value.
#pragma once

namespace cardLib {

// v_k = alpha * raw + (1 - alpha) * v_(k-1). alpha = 1 means no smoothing.
class EmaFilter {
 public:
  explicit EmaFilter(double alpha) : m_alpha(alpha) {}

  double update(double raw) {
    m_value = m_alpha * raw + (1.0 - m_alpha) * m_value;
    return m_value;
  }

  void reset(double value = 0.0) { m_value = value; }
  double value() const { return m_value; }

 private:
  double m_alpha;
  double m_value = 0.0;
};

}  // namespace cardLib
