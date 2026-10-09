// reference_state.hpp
// Why: where the robot should be right now, for profiled motions and state feedback (SPEC 18).
// Owns: ReferenceState. Plain data; nothing in v1 produces it yet.
#pragma once

#include "cardLib/algorithms/math/pose.hpp"

namespace cardLib {

struct ReferenceState {
  Pose pose = {};
  double linearVelocityInchesPerSecond = 0.0;
  double angularVelocityDegreesPerSecond = 0.0;
};

}  // namespace cardLib
