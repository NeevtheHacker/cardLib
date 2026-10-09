// drive_pipeline.cpp
// Why: implements SPEC section 7 steps 2 to 6.
// Owns: nothing beyond DrivePipeline's two slew limiters.
#include "cardLib/algorithms/control/drive_pipeline.hpp"

#include <algorithm>
#include <cmath>

#include "cardLib/algorithms/control/drive_mixer.hpp"

namespace cardLib {

double limitSpeed(const ChannelRequest& request) {
  const SpeedLimits& limits = request.limits;
  const double clamped = std::clamp(request.speed, -limits.maxSpeed, limits.maxSpeed);
  const bool wantsMinSpeed = limits.minSpeed > 0.0 && std::abs(clamped) < limits.minSpeed &&
                             std::abs(request.error) > limits.minSpeedCutoff;
  if (!wantsMinSpeed) {
    return clamped;
  }
  return std::copysign(limits.minSpeed, request.error);
}

DrivePipeline::DrivePipeline(const SlewRates& linearRates, const SlewRates& angularRates)
    : m_linearSlew(linearRates), m_angularSlew(angularRates) {}

DriveCommand DrivePipeline::update(const ChannelRequest& linear, const ChannelRequest& angular,
                                   double dtSeconds) {
  const double linearSpeed = m_linearSlew.update(limitSpeed(linear), dtSeconds);
  const double angularSpeed = m_angularSlew.update(limitSpeed(angular), dtSeconds);
  const SideSpeeds sides = mixDrive(linearSpeed, angularSpeed);
  return {.leftMillivolts = speedToMillivolts(sides.left),
          .rightMillivolts = speedToMillivolts(sides.right)};
}

DriveCommand DrivePipeline::updatePivot(const ChannelRequest& turn, Side lockedSide,
                                        double dtSeconds) {
  m_linearSlew.reset();
  const std::int32_t millivolts =
      speedToMillivolts(m_angularSlew.update(limitSpeed(turn), dtSeconds));
  if (lockedSide == Side::Left) {
    return {.leftMillivolts = 0,
            .rightMillivolts = -millivolts,
            .leftMode = SideMode::Held,
            .rightMode = SideMode::Driven};
  }
  return {.leftMillivolts = millivolts,
          .rightMillivolts = 0,
          .leftMode = SideMode::Driven,
          .rightMode = SideMode::Held};
}

void DrivePipeline::reset(double linearSpeed, double angularSpeed) {
  m_linearSlew.reset(linearSpeed);
  m_angularSlew.reset(angularSpeed);
}

void DrivePipeline::setRates(const SlewRates& linearRates, const SlewRates& angularRates) {
  m_linearSlew.setRates(linearRates);
  m_angularSlew.setRates(angularRates);
}

}  // namespace cardLib
