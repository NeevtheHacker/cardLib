// motion_options.hpp
// Why: named options instead of long argument lists, e.g. {.maxSpeed = 80, .timeoutMs = 1500}.
// Owns: MotionOptions, shared by every chassis motion (SPEC sections 7 to 9).
#pragma once

#include <cstdint>
#include <optional>

#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

// Used when a motion's timeoutMs is not set and the chassis hasn't supplied its own default.
constexpr std::uint32_t DEFAULT_TIMEOUT_MS = 3000;

struct MotionOptions {
  double maxSpeed = 127.0;
  double minSpeed = 0.0;   // with earlyExit, lets chained motions flow without stopping
  double earlyExit = 0.0;  // inches or degrees; 0 = off
  std::optional<std::uint32_t> timeoutMs = std::nullopt;  // empty = the chassis default
  bool forwards = true;  // false drives backwards (moves) or faces away (turnToPoint)
  TurnDirection direction = TurnDirection::Shortest;
  bool async = false;          // return MotionResult::Started at once
  bool allowDegraded = false;  // run even when odometry is on a fallback
  // Holonomic moveToPoint: the heading to hold; empty = the heading it started at (SPEC 20.10).
  std::optional<double> headingDegrees = std::nullopt;
  bool tankStyle = false;  // holonomic drive: run moveToPoint / moveToPose the tank way
};

}  // namespace cardLib
