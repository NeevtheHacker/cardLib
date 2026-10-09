// motion_command.hpp
// Why: what user code hands the motion task: one command slot, read under a mutex (SPEC 2).
// Owns: MotionKind, MotionCommand, DriverCommand and ReadyState. Plain data.
#pragma once

#include <cstdint>

#include "cardLib/algorithms/control/drive_command.hpp"
#include "cardLib/algorithms/math/pose.hpp"
#include "cardLib/algorithms/motion/motion_options.hpp"

namespace cardLib {

enum class MotionKind { None, Move, Turn, TurnToPoint, PivotTurn, MoveToPoint, MoveToPose };

struct MotionCommand {
  MotionKind kind = MotionKind::None;
  double distanceInches = 0.0;  // move
  Pose target = {};             // turn: heading; point motions: x, y; moveToPose: all three
  Side lockedSide = Side::Left;  // pivotTurn
  MotionOptions options = {};
};

// Driver control output, already curved and mixed.
struct DriverCommand {
  std::int32_t leftMillivolts = 0;
  std::int32_t rightMillivolts = 0;
};

// Whether startup validation lets motions run (SPEC section 3).
enum class ReadyState { Ready, NotCalibrated, SensorFault };

}  // namespace cardLib
