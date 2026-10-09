// drive_curve.hpp
// Why: gentle joystick response near center, full power at the edges (SPEC section 10).
// Owns: the deadzone and expo curve. No state.
#pragma once

namespace cardLib {

// One curve per stick axis (throttle and turn get separate ones).
struct DriveCurve {
  double deadzone = 3.0;   // |input| below this becomes 0 (tune)
  double curveGain = 0.0;  // t: 0 = linear, bigger = gentler at low stick
};

// out = x * (e^(-t/10) + e^((|x| - 127)/10) * (1 - e^(-t/10))), on the -127 to 127 scale.
double applyDriveCurve(double input, const DriveCurve& curve);

}  // namespace cardLib
