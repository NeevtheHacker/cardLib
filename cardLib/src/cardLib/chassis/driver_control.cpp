// driver_control.cpp
// Why: implements tank and arcade from SPEC section 10.
// Owns: nothing beyond DriverControl's curves.
#include "cardLib/chassis/driver_control.hpp"

#include "cardLib/algorithms/control/drive_mixer.hpp"

namespace cardLib {

DriverCommand DriverControl::tank(double leftStick, double rightStick) const {
  return {speedToMillivolts(applyDriveCurve(leftStick, m_throttleCurve)),
          speedToMillivolts(applyDriveCurve(rightStick, m_throttleCurve))};
}

// Arcade uses the same desaturation as autonomous, so a full-throttle turn still turns.
DriverCommand DriverControl::arcade(double forwardStick, double turnStick) const {
  const SideSpeeds sides = mixDrive(applyDriveCurve(forwardStick, m_throttleCurve),
                                    applyDriveCurve(turnStick, m_turnCurve));
  return {speedToMillivolts(sides.left), speedToMillivolts(sides.right)};
}

void DriverControl::setCurves(const DriveCurve& throttleCurve, const DriveCurve& turnCurve) {
  m_throttleCurve = throttleCurve;
  m_turnCurve = turnCurve;
}

}  // namespace cardLib
