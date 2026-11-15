// motion_command.hpp
// Why: what user code hands the motion task: one command slot, read under a mutex (SPEC 2).
// Owns: MotionKind, MotionCommand, DriverCommand and ReadyState. Plain data.
#pragma once

#include <cstdint>

#include "cardLib/algorithms/control/drive_command.hpp"
#include "cardLib/algorithms/control/drive_kinematics.hpp"
#include "cardLib/algorithms/math/pose.hpp"
#include "cardLib/algorithms/motion/motion_options.hpp"

namespace cardLib {

enum class MotionKind {
  None,
  Move,
  Turn,
  TurnToPoint,
  PivotTurn,
  MoveToPoint,
  MoveToPose,
  Strafe,
};

struct MotionCommand {
  MotionKind kind = MotionKind::None;
  double distanceInches = 0.0;  // move, strafe
  Pose target = {};             // turn: heading; point motions: x, y; moveToPose: all three
  Side lockedSide = Side::Left;  // pivotTurn
  bool pivotAboutPoint = false;  // pivotTurn about pivotPoint instead of a side (SPEC 20.9)
  LocalOffset pivotPoint = {};
  MotionOptions options = {};
};

// Driver control output, already curved. Either speeds the motion task mixes through the drive's
// kinematics (arcade, holonomic), or a ready command (tank sticks, raw test voltages).
struct DriverCommand {
  bool mixSpeeds = false;
  DriveSpeeds speeds = {};
  DriveCommand direct = {};
};

// Whether startup validation lets motions run (SPEC section 3).
enum class ReadyState { Ready, NotCalibrated, SensorFault };

}  // namespace cardLib
