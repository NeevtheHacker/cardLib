// line_fit.hpp
// Why: turns sensor readings at known distances into a gain and offset (SPEC section 6).
// Owns: the least-squares line fit. No state.
#pragma once

#include <optional>
#include <span>

namespace cardLib {

// true = gain * raw + offset
struct LineFit {
  double gain = 1.0;
  double offset = 0.0;
};

// Least-squares fit of truth against raw. Empty if the spans differ in length, there are fewer
// than 2 points, or every raw value is the same.
std::optional<LineFit> fitLine(std::span<const double> raw, std::span<const double> truth);

}  // namespace cardLib
