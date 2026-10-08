// mechanism_controller.cpp
// Why: implements the SPEC section 7 mechanism control: PID + kG, soft limits, hard stops.
// Owns: nothing beyond MechanismController's PID.
#include "cardLib/algorithms/control/mechanism_controller.hpp"

#include <algorithm>
#include <cmath>

#include "cardLib/algorithms/control/drive_mixer.hpp"
#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

MechanismController::MechanismController(const MechanismConfig& config)
    : m_config(config), m_pid(config.gains) {}

double MechanismController::update(double target, double position, double dtSeconds) {
  const double error = clampTarget(target) - position;
  const double output = std::clamp(m_pid.update(error, dtSeconds) + gravityHold(position),
                                   -MAX_SPEED, MAX_SPEED);
  if (pushesIntoHardStop(output, position)) {
    return 0.0;
  }
  return output;
}

double MechanismController::clampTarget(double target) const {
  return std::clamp(target, m_config.minTarget, m_config.maxTarget);
}

double MechanismController::gravityHold(double position) const {
  switch (m_config.gravity) {
    case GravityMode::Arm:
      return m_config.gravityGain * std::cos(degreesToRadians(position));
    case GravityMode::Lift:
      return m_config.gravityGain;
    case GravityMode::None:
      break;
  }
  return 0.0;
}

bool MechanismController::pushesIntoHardStop(double output, double position) const {
  const bool nearMax = position >= m_config.hardStopMax - m_config.hardStopBand;
  const bool nearMin = position <= m_config.hardStopMin + m_config.hardStopBand;
  return (nearMax && output > 0.0) || (nearMin && output < 0.0);
}

}  // namespace cardLib
