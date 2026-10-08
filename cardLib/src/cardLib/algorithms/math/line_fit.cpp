// line_fit.cpp
// Why: implements the SPEC section 6 least-squares line fit.
// Owns: nothing beyond the math; no state.
#include "cardLib/algorithms/math/line_fit.hpp"

#include <cstddef>

namespace cardLib {

std::optional<LineFit> fitLine(std::span<const double> raw, std::span<const double> truth) {
  const std::size_t count = raw.size();
  if (count != truth.size() || count < 2) {
    return std::nullopt;
  }

  double rawSum = 0.0;
  double truthSum = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    rawSum += raw[i];
    truthSum += truth[i];
  }
  const double rawMean = rawSum / static_cast<double>(count);
  const double truthMean = truthSum / static_cast<double>(count);

  double covariance = 0.0;
  double rawVariance = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    covariance += (raw[i] - rawMean) * (truth[i] - truthMean);
    rawVariance += (raw[i] - rawMean) * (raw[i] - rawMean);
  }
  if (rawVariance == 0.0) {
    return std::nullopt;
  }

  const double gain = covariance / rawVariance;
  return LineFit{gain, truthMean - gain * rawMean};
}

}  // namespace cardLib
