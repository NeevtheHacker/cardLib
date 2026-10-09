// loop_timing.hpp
// Why: the control loop period and the overrun rule (SPEC sections 2 and 7) in one place.
// Owns: LOOP_PERIOD_MS, LOOP_OVERRUN_FACTOR and isLoopOverrun().
#pragma once

#include <cstdint>

namespace cardLib {

// Odometry and motion tasks both run every 10 ms.
constexpr std::uint32_t LOOP_PERIOD_MS = 10;
constexpr double LOOP_PERIOD_SECONDS = LOOP_PERIOD_MS / 1000.0;

// A loop whose measured dt is more than this many periods skips D and adds nothing to I. (tune)
constexpr double LOOP_OVERRUN_FACTOR = 2.5;

// True when dt is too long (or not positive) to trust a derivative or integral step.
constexpr bool isLoopOverrun(double dtSeconds) {
  return dtSeconds <= 0.0 || dtSeconds > LOOP_OVERRUN_FACTOR * LOOP_PERIOD_SECONDS;
}

}  // namespace cardLib
