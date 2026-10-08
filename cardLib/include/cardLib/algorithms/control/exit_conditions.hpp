// exit_conditions.hpp
// Why: decides when a motion is done and why, in the SPEC section 9 order.
// Owns: the timers for the small, large and stall windows.
#pragma once

#include <cstdint>
#include <optional>
#include <span>

#include "cardLib/algorithms/motion/motion_result.hpp"

namespace cardLib {

// Error in inches or degrees; velocities in in/s or deg/s. (tune)
struct ExitWindows {
  double smallError = 0.0;
  std::uint32_t smallTimeMs = 0;
  double largeError = 0.0;
  std::uint32_t largeTimeMs = 0;
  double velocityTolerance = 0.0;
  double stallSpeed = 0.0;
  std::uint32_t stallTimeMs = 0;
};

constexpr ExitWindows MOVE_EXIT_WINDOWS{1.0, 100, 3.0, 500, 2.0, 1.0, 300};
constexpr ExitWindows TURN_EXIT_WINDOWS{1.0, 100, 3.0, 500, 5.0, 2.5, 300};
constexpr ExitWindows PIVOT_EXIT_WINDOWS{1.5, 100, 4.0, 500, 5.0, 2.5, 300};

struct ExitSettings {
  ExitWindows windows = MOVE_EXIT_WINDOWS;
  double earlyExit = 0.0;  // 0 = off
  std::uint32_t timeoutMs = 3000;
  bool useCurrentForStall = false;
};

// What the motion measured this loop.
struct ExitInputs {
  double error = 0.0;
  double velocity = 0.0;
  std::uint32_t elapsedMs = 0;
  bool calibrated = true;
  bool sensorFault = false;
  bool cancelled = false;
  bool currentConfirmsStall = false;  // only read when useCurrentForStall is on
};

struct ExitDecision {
  MotionResult result = MotionResult::Settled;
  bool brake = true;  // false only for earlyExit, so a chained motion flows on
};

class ExitConditions {
 public:
  explicit ExitConditions(const ExitSettings& settings) : m_settings(settings) {}

  // Clears every window timer. Call at the start of each motion (or phase).
  void reset();
  void reset(const ExitSettings& settings);

  // Empty while the motion should keep running.
  std::optional<ExitDecision> check(const ExitInputs& inputs);

 private:
  // Tracks how long a condition has held without a break.
  class HeldTimer {
   public:
    bool heldFor(bool condition, std::uint32_t nowMs, std::uint32_t durationMs);
    void reset() { m_active = false; }

   private:
    bool m_active = false;
    std::uint32_t m_sinceMs = 0;
  };

  ExitSettings m_settings;
  HeldTimer m_smallTimer;
  HeldTimer m_largeTimer;
  HeldTimer m_stallTimer;
};

// Stall by motor current: at least half the motors over the threshold, and the commanded speed
// above a minimum so a robot quietly holding still can't count.
bool currentConfirmsStall(std::span<const double> currentsAmps, double thresholdAmps,
                          double commandedSpeed, double minCommandedSpeed);

}  // namespace cardLib
