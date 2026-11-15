// drive_command.hpp
// Why: what a motion asks the drive to do this loop, in the units the motors take.
// Owns: DriveCommand, the per-side and per-wheel mode, and the speed and voltage limits.
//       Plain data.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "cardLib/algorithms/control/wheel_module_limit.hpp"

namespace cardLib {

constexpr double MAX_SPEED = 127.0;
constexpr std::int32_t MAX_MILLIVOLTS = 12000;

// A side of the drivetrain, e.g. the locked side of a pivot turn.
enum class Side { Left, Right };

// Driven: apply the voltage. Held: stop with brake mode Hold (the locked side of a pivot turn,
// or any wheel a pivot doesn't move).
enum class SideMode { Driven, Held };

struct WheelCommand {
  std::int32_t millivolts = 0;
  SideMode mode = SideMode::Driven;
};

// wheelCount == 0: a two-side command; leftMillivolts goes to every left wheel and
// rightMillivolts to every right wheel (tank sticks, raw test voltages).
// wheelCount > 0: wheels[] is the command, one entry per wheel module in the drive's order, and
// the side fields are a summary (the mean of the left and of the right wheels) for the screen,
// telemetry and the stall check.
struct DriveCommand {
  std::int32_t leftMillivolts = 0;
  std::int32_t rightMillivolts = 0;
  SideMode leftMode = SideMode::Driven;
  SideMode rightMode = SideMode::Driven;
  std::array<WheelCommand, MAX_WHEEL_MODULES> wheels = {};
  std::size_t wheelCount = 0;
};

}  // namespace cardLib
