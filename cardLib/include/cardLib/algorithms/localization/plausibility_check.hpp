// plausibility_check.hpp
// Why: turns one sensor's running total into a per-loop change that never invents motion (SPEC 4).
// Owns: that sensor's baseline, rejection count and fault flag.
#pragma once

#include <cstdint>
#include <optional>

#include "cardLib/algorithms/localization/sensor_frame.hpp"

namespace cardLib {

class PlausibilityCheck {
 public:
  // maxSpeed: the chassis' top speed, pi * D * rpm / 60 of the drive wheels.
  explicit PlausibilityCheck(double maxSpeedInchesPerSecond, double margin = 1.5)
      : m_maxSpeedInchesPerSecond(maxSpeedInchesPerSecond), m_margin(margin) {}

  // totalInches: the sensor's cumulative distance, or empty if the read failed. Returns the
  // change to use this loop: 0 on the first reading, on a failed or implausible reading, and
  // on the first good reading after one (which only re-baselines).
  double update(std::optional<double> totalInches, double dtSeconds);

  // update(), packaged for odometry: the change, and whether the sensor is usable. A single
  // failed or rejected reading (and the re-baseline after it) is a usable zero; only a fault
  // makes the sensor unusable (SPEC section 4).
  WheelDelta measure(std::optional<double> totalInches, double dtSeconds);

  // Fresh baseline and a cleared fault, as setPose does.
  void rebaseline(std::optional<double> totalInches);

  // True only when this loop's change was actually measured: false on a rejected or failed
  // read and on a re-baseline, when the change returned is 0 but the wheel may have moved.
  bool lastChangeMeasured() const { return m_lastChangeMeasured; }

  // Flagged: this loop's reading was rejected. Faulted: three rejections in a row; stays set
  // until rebaseline().
  bool flagged() const { return m_flagged; }
  bool faulted() const { return m_faulted; }

  // How many readings have been rejected or failed since start-up.
  std::uint32_t totalRejections() const { return m_totalRejections; }

 private:
  double reject();

  double m_maxSpeedInchesPerSecond;
  double m_margin;
  double m_lastInches = 0.0;
  bool m_hasBaseline = false;
  bool m_flagged = false;
  bool m_lastChangeMeasured = false;
  bool m_faulted = false;
  std::uint32_t m_rejectionsInARow = 0;
  std::uint32_t m_totalRejections = 0;
};

}  // namespace cardLib
