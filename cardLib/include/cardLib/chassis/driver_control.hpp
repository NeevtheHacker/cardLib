// driver_control.hpp
// Why: turns joystick values into drive voltages: deadzone, expo, arcade mixing (SPEC 10).
// Owns: the throttle and turn curves. The motion task applies the result, so driver control
//       and autonomous never fight over the motors.
#pragma once

#include "cardLib/algorithms/control/drive_curve.hpp"
#include "cardLib/chassis/motion_command.hpp"

namespace cardLib {

class DriverControl {
 public:
  DriverControl(const DriveCurve& throttleCurve, const DriveCurve& turnCurve)
      : m_throttleCurve(throttleCurve), m_turnCurve(turnCurve) {}

  // Sticks on the -127 to 127 scale.
  DriverCommand tank(double leftStick, double rightStick) const;
  DriverCommand arcade(double forwardStick, double turnStick) const;

  void setCurves(const DriveCurve& throttleCurve, const DriveCurve& turnCurve);

 private:
  DriveCurve m_throttleCurve;
  DriveCurve m_turnCurve;
};

}  // namespace cardLib
