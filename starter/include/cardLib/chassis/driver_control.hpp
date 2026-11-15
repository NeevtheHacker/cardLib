// driver_control.hpp
// Why: turns joystick values into drive voltages: deadzone, expo, arcade mixing (SPEC 10).
// Owns: the throttle and turn curves. The motion task applies the result, so driver control
//       and autonomous never fight over the motors.
#pragma once

#include <optional>

#include "cardLib/algorithms/control/drive_curve.hpp"
#include "cardLib/chassis/motion_command.hpp"

namespace cardLib {

// Robot-centric: stick forward moves the way the robot faces. Field-centric: stick forward
// always moves away from the driver (SPEC 20.11). Robot-centric is the default.
enum class DriveMode { RobotCentric, FieldCentric };

// Per-call options for Chassis::holonomic(). fieldCentric overrides the drive mode for this call.
struct HolonomicOptions {
  std::optional<bool> fieldCentric = std::nullopt;
};

class DriverControl {
 public:
  DriverControl(const DriveCurve& throttleCurve, const DriveCurve& turnCurve)
      : m_throttleCurve(throttleCurve), m_turnCurve(turnCurve) {}

  // Sticks on the -127 to 127 scale.
  DriverCommand tank(double leftStick, double rightStick) const;
  DriverCommand arcade(double forwardStick, double turnStick) const;
  // Robot-centric: forward and strafe each use the throttle curve, turn the turn curve. On a
  // tank the strafe stick does nothing.
  DriverCommand holonomic(double forwardStick, double strafeStick, double turnStick) const;
  // Field-centric: the throttle curve shapes the stick's length, then the stick is turned by
  // headingDegrees, so the feel doesn't change with the way the robot faces.
  DriverCommand holonomicFieldCentric(double forwardStick, double strafeStick, double turnStick,
                                      double headingDegrees) const;

  void setCurves(const DriveCurve& throttleCurve, const DriveCurve& turnCurve);

 private:
  DriveCurve m_throttleCurve;
  DriveCurve m_turnCurve;
};

}  // namespace cardLib
