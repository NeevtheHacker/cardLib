// mechanism_controller.hpp
// Why: the same PID for arms and lifts, plus gravity hold, soft limits and hard stops (SPEC 7).
// Owns: the mechanism's PID and its config.
#pragma once

#include <limits>

#include "cardLib/algorithms/control/pid_controller.hpp"

namespace cardLib {

// Arm: kG * cos(position), position in degrees from horizontal. Lift: constant kG.
enum class GravityMode { None, Arm, Lift };

struct MechanismConfig {
  PidGains gains = {};
  GravityMode gravity = GravityMode::None;
  double gravityGain = 0.0;  // kG, speed units: the smallest output that holds it level (tune)
  double minTarget = -std::numeric_limits<double>::infinity();  // soft limits
  double maxTarget = std::numeric_limits<double>::infinity();
  double hardStopMin = -std::numeric_limits<double>::infinity();
  double hardStopMax = std::numeric_limits<double>::infinity();
  double hardStopBand = 0.0;  // output pushing into a stop is cut this close to it
};

class MechanismController {
 public:
  explicit MechanismController(const MechanismConfig& config);

  // Output on the -127 to 127 scale. Positive output moves toward larger positions.
  double update(double target, double position, double dtSeconds);

  void reset() { m_pid.reset(); }
  double clampTarget(double target) const;
  void setGains(const PidGains& gains) { m_pid.setGains(gains); }

 private:
  double gravityHold(double position) const;
  bool pushesIntoHardStop(double output, double position) const;

  MechanismConfig m_config;
  PidController m_pid;
};

}  // namespace cardLib
