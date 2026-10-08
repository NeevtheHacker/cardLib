// heading_disagreement.hpp
// Why: notices when wheel heading and IMU heading drift apart (slip, lifted wheel) (SPEC 4).
// Owns: the last 10 loops of disagreement. It only raises a flag; it never edits the pose.
#pragma once

#include <array>
#include <cstddef>

namespace cardLib {

class HeadingDisagreement {
 public:
  static constexpr std::size_t WINDOW_LOOPS = 10;  // 100 ms at 10 ms per loop

  explicit HeadingDisagreement(double thresholdDegrees = 2.0)  // (tune)
      : m_thresholdDegrees(thresholdDegrees) {}

  // Adds this loop's d = wrap(wheel change - IMU change). Suspect while the sum over the window
  // is above the threshold.
  bool update(double wheelChangeDegrees, double imuChangeDegrees);

  bool suspect() const { return m_suspect; }
  void reset();

 private:
  double m_thresholdDegrees;
  std::array<double, WINDOW_LOOPS> m_differences = {};
  std::size_t m_next = 0;
  bool m_suspect = false;
};

}  // namespace cardLib
