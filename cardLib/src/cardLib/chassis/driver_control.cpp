// driver_control.cpp
// Why: implements tank, arcade and holonomic driving from SPEC sections 10 and 20.11.
// Owns: nothing beyond DriverControl's curves.
#include "cardLib/chassis/driver_control.hpp"

#include "cardLib/algorithms/control/drive_mixer.hpp"
#include "cardLib/algorithms/control/field_centric.hpp"

namespace cardLib {

// Each stick drives its side directly: every left wheel and every right wheel (SPEC 20.3).
DriverCommand DriverControl::tank(double leftStick, double rightStick) const {
  const double left = applyDriveCurve(leftStick, m_throttleCurve);
  const double right = applyDriveCurve(rightStick, m_throttleCurve);
  return {.direct = {.leftMillivolts = speedToMillivolts(left),
                     .rightMillivolts = speedToMillivolts(right)}};
}

// Arcade uses the same desaturation as autonomous (mixed by the motion task through the drive's
// kinematics), so a full-throttle turn still turns.
DriverCommand DriverControl::arcade(double forwardStick, double turnStick) const {
  return {.mixSpeeds = true,
          .speeds = {.forward = applyDriveCurve(forwardStick, m_throttleCurve),
                     .turn = applyDriveCurve(turnStick, m_turnCurve)}};
}

DriverCommand DriverControl::holonomic(double forwardStick, double strafeStick,
                                       double turnStick) const {
  return {.mixSpeeds = true,
          .speeds = {.forward = applyDriveCurve(forwardStick, m_throttleCurve),
                     .strafe = applyDriveCurve(strafeStick, m_throttleCurve),
                     .turn = applyDriveCurve(turnStick, m_turnCurve)}};
}

DriverCommand DriverControl::holonomicFieldCentric(double forwardStick, double strafeStick,
                                                   double turnStick, double headingDegrees) const {
  const StickVector robot =
      curvedFieldCentricSticks(forwardStick, strafeStick, headingDegrees, m_throttleCurve);
  return {.mixSpeeds = true,
          .speeds = {.forward = robot.forward,
                     .strafe = robot.strafe,
                     .turn = applyDriveCurve(turnStick, m_turnCurve)}};
}

void DriverControl::setCurves(const DriveCurve& throttleCurve, const DriveCurve& turnCurve) {
  m_throttleCurve = throttleCurve;
  m_turnCurve = turnCurve;
}

}  // namespace cardLib
