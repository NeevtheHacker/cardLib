// field_centric.hpp
// Why: field-centric driving turns the sticks by the heading, so stick forward always moves the
//      robot away from the driver (SPEC 20.11).
// Owns: fieldCentricSticks(). Uses fieldToLocal, so the sign lives in one place.
#pragma once

#include "cardLib/algorithms/control/drive_curve.hpp"

namespace cardLib {

// Sticks on the -127 to 127 scale: forward and strafe (right).
struct StickVector {
  double forward = 0.0;
  double strafe = 0.0;
};

// The field-frame sticks in the robot frame, for a robot facing headingDegrees from the
// field-centric forward: forward = s_x sin(theta) + s_y cos(theta),
// right = s_x cos(theta) - s_y sin(theta).
StickVector fieldCentricSticks(double forwardStick, double strafeStick, double headingDegrees);

StickVector curvedFieldCentricSticks(double forwardStick, double strafeStick, double headingDegrees,
                                     const DriveCurve& curve);

}  // namespace cardLib
