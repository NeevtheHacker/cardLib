// robot_state.hpp
// Why: what every motion reads each loop: pose, velocities and how much to trust them (SPEC 18).
// Owns: RobotState and OdometryHealth. Plain data.
#pragma once

#include "cardLib/algorithms/math/pose.hpp"

namespace cardLib {

// Ok: every configured sensor is working. Degraded: running on a fallback (SPEC section 4).
// Faulted: no heading or no forward source at all; the pose can't be trusted.
enum class OdometryHealth { Ok, Degraded, Faulted };

struct RobotState {
  Pose pose = {};
  double linearVelocityInchesPerSecond = 0.0;    // along the robot's heading
  double sidewaysVelocityInchesPerSecond = 0.0;  // to the robot's right (SPEC 20.12)
  double angularVelocityDegreesPerSecond = 0.0;  // clockwise positive
  OdometryHealth health = OdometryHealth::Ok;
  bool wheelHeadingSuspect = false;  // wheels and IMU disagree (slip or a lifted wheel)
  bool driveSlipSuspect = false;     // drive wheels disagree with each other (holonomic only)
};

}  // namespace cardLib
