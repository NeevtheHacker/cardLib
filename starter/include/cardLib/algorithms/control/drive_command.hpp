// drive_command.hpp
// Why: what a motion asks the drive to do this loop, in the units the motors take.
// Owns: DriveCommand and the per-side mode. Plain data.
#pragma once

#include <cstdint>

namespace cardLib {

// A side of the drivetrain, e.g. the locked side of a pivot turn.
enum class Side { Left, Right };

// Driven: apply the voltage. Held: stop with brake mode Hold (the locked side of a pivot turn).
enum class SideMode { Driven, Held };

struct DriveCommand {
  std::int32_t leftMillivolts = 0;
  std::int32_t rightMillivolts = 0;
  SideMode leftMode = SideMode::Driven;
  SideMode rightMode = SideMode::Driven;
};

}  // namespace cardLib
