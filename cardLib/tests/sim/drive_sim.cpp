// drive_sim.cpp
// Why: implements the SPEC section 13 first-order drive model.
// Owns: nothing beyond DriveSim's own state.
#include "drive_sim.hpp"

#include "cardLib/algorithms/control/drive_mixer.hpp"
#include "cardLib/algorithms/localization/wheel_distance.hpp"

namespace cardLib::sim {

DriveSim::DriveSim(const DriveSimConfig& config, const Pose& start)
    : m_config(config), m_truth({.effectiveTrackWidthInches = config.trackWidthInches}) {
  m_truth.reset(start);
}

double DriveSim::nextSpeed(double speed, std::int32_t millivolts, SideMode mode,
                           double dtSeconds) const {
  if (mode == SideMode::Held) {
    return 0.0;
  }
  const double freeSpeed =
      maxWheelSpeedInchesPerSecond(m_config.wheelDiameterInches, m_config.wheelRpm);
  const double targetSpeed = freeSpeed * millivolts / static_cast<double>(MAX_MILLIVOLTS);
  return speed + (targetSpeed - speed) * dtSeconds / m_config.timeConstantSeconds;
}

void DriveSim::advance(double dtSeconds) {
  m_leftSpeed = nextSpeed(m_leftSpeed, m_command.leftMillivolts, m_command.leftMode, dtSeconds);
  m_rightSpeed = nextSpeed(m_rightSpeed, m_command.rightMillivolts, m_command.rightMode, dtSeconds);

  SensorFrame motion;
  motion.dtSeconds = dtSeconds;
  motion.leftDrive = {m_leftSpeed * dtSeconds, true};
  motion.rightDrive = {m_rightSpeed * dtSeconds, true};
  m_truth.update(motion);
  m_leftTotalInches += motion.leftDrive.inches;
  m_rightTotalInches += motion.rightDrive.inches;
}

}  // namespace cardLib::sim
