// drive_slip_check.hpp
// Why: notices when a drive with more wheels than it needs has wheels that disagree with each
//      other (one slipping or lifted), from the forward-kinematics residual (SPEC 20.12).
// Owns: the last 10 loops of residual. It only raises a flag; it never edits the pose.
#pragma once

#include <array>
#include <cstddef>

namespace cardLib {

class DriveSlipCheck {
 public:
  static constexpr std::size_t WINDOW_LOOPS = 10;  // 100 ms at 10 ms per loop

  explicit DriveSlipCheck(double thresholdInches = 0.5)  // (tune)
      : m_thresholdInches(thresholdInches) {}

  // Adds this loop's residual (RMS wheel travel the fit can't explain, inches). Suspect while
  // the sum over the window is above the threshold.
  bool update(double residualInches);

  bool suspect() const { return m_suspect; }
  void reset();

 private:
  double m_thresholdInches;
  std::array<double, WINDOW_LOOPS> m_residuals = {};
  std::size_t m_next = 0;
  bool m_suspect = false;
};

}  // namespace cardLib
