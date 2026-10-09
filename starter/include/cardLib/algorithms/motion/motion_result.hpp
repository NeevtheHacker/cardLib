// motion_result.hpp
// Why: every motion ends with one of these, so autons can decide what to do next (SPEC 9).
// Owns: the MotionResult enum.
#pragma once

namespace cardLib {

enum class MotionResult {
  Started,        // returned by an async motion right away; waitUntilDone() gives the real result
  Settled,        // reached the target
  Stalled,        // stopped short and isn't moving
  TimedOut,       // ran out of time
  Cancelled,      // cancel() was called, or driver control took over
  SensorFault,    // a required sensor failed with no fallback, or odometry is degraded
  NotCalibrated,  // startup validation hasn't passed
};

}  // namespace cardLib
