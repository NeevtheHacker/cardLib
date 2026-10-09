// plausibility_check.hpp
// Why: turns one sensor's running total into a per-loop change that never invents motion (SPEC 4).
// Owns: that sensor's baseline, rejection count and fault flag.
#pragma once

#include <cstdint>
#include <optional>

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

  // Fresh baseline and a cleared fault, as setPose does.
  void rebaseline(std::optional<double> totalInches);

  // True only when this loop's change was actually measured: false on a rejected or failed
  // read and on a re-baseline, when the change returned is 0 but the wheel may have moved.
  bool lastChangeMeasured() const { return m_lastChangeMeasured; }

  // Flagged: this loop's reading was rejected. Faulted: three rejections in a row; stays set
  // until rebaseline().
  bool flagged() const { return m_flagged; }
  bool faulted() const { return m_faulted; }

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
};

}  // namespace cardLib
