// drive_curve.cpp
// Why: implements the SPEC section 10 deadzone and expo curve.
// Owns: nothing; no state.
#include "cardLib/algorithms/control/drive_curve.hpp"

#include <algorithm>
#include <cmath>

#include "cardLib/algorithms/control/drive_mixer.hpp"

namespace cardLib {

double applyDriveCurve(double input, const DriveCurve& curve) {
  const double stick = std::clamp(input, -MAX_SPEED, MAX_SPEED);
  if (std::abs(stick) < curve.deadzone) {
    return 0.0;
  }
  const double lowStickWeight = std::exp(-curve.curveGain / 10.0);
  const double edgeWeight = std::exp((std::abs(stick) - MAX_SPEED) / 10.0);
  return stick * (lowStickWeight + edgeWeight * (1.0 - lowStickWeight));
}

}  // namespace cardLib
