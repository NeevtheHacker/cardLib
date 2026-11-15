// drive_pipeline.cpp
// Why: implements SPEC section 7 steps 2 to 6, mixed per wheel through the drive's kinematics
//      (SPEC 20.8, 20.9).
// Owns: nothing beyond DrivePipeline's kinematics and slew limiters.
#include "cardLib/algorithms/control/drive_pipeline.hpp"

#include <algorithm>
#include <cmath>

#include "cardLib/algorithms/control/drive_mixer.hpp"

namespace cardLib {

namespace {

// The rounded mean of some wheels' millivolts, and whether every one of them is held.
struct SideSummary {
  std::int32_t millivolts = 0;
  SideMode mode = SideMode::Driven;
};

SideSummary summarizeSide(const DriveCommand& command, const DriveKinematics& kinematics,
                          bool leftSide) {
  std::int64_t sum = 0;
  std::size_t count = 0;
  bool allHeld = true;
  for (std::size_t i = 0; i < command.wheelCount; ++i) {
    const double x = kinematics.wheel(i).xInches;
    if ((leftSide && x < 0.0) || (!leftSide && x > 0.0)) {
      sum += command.wheels[i].millivolts;
      allHeld = allHeld && command.wheels[i].mode == SideMode::Held;
      ++count;
    }
  }
  if (count == 0) {
    return {};
  }
  return {static_cast<std::int32_t>(std::lround(static_cast<double>(sum) / count)),
          allHeld ? SideMode::Held : SideMode::Driven};
}

}  // namespace

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

DriveCommand toDriveCommand(const WheelSpeeds& wheels, const DriveKinematics& kinematics) {
  DriveCommand command;
  command.wheelCount = std::min(wheels.count, kinematics.wheelCount());
  for (std::size_t i = 0; i < command.wheelCount; ++i) {
    command.wheels[i] = wheels.held[i]
                            ? WheelCommand{0, SideMode::Held}
                            : WheelCommand{speedToMillivolts(wheels.speeds[i]), SideMode::Driven};
  }
  summarizeSides(command, kinematics);
  return command;
}

void summarizeSides(DriveCommand& command, const DriveKinematics& kinematics) {
  const SideSummary left = summarizeSide(command, kinematics, true);
  const SideSummary right = summarizeSide(command, kinematics, false);
  command.leftMillivolts = left.millivolts;
  command.leftMode = left.mode;
  command.rightMillivolts = right.millivolts;
  command.rightMode = right.mode;
}

double drivenMillivoltMagnitude(const DriveCommand& command) {
  double sum = 0.0;
  std::size_t driven = 0;
  if (command.wheelCount == 0) {
    if (command.leftMode == SideMode::Driven) {
      sum += std::abs(command.leftMillivolts);
      ++driven;
    }
    if (command.rightMode == SideMode::Driven) {
      sum += std::abs(command.rightMillivolts);
      ++driven;
    }
  }
  for (std::size_t i = 0; i < command.wheelCount; ++i) {
    if (command.wheels[i].mode == SideMode::Driven) {
      sum += std::abs(command.wheels[i].millivolts);
      ++driven;
    }
  }
  return driven == 0 ? 0.0 : sum / static_cast<double>(driven);
}

DrivePipeline::DrivePipeline(const SlewRates& linearRates, const SlewRates& angularRates)
    : DrivePipeline(UNIT_TANK_KINEMATICS, linearRates, linearRates, angularRates) {}

DrivePipeline::DrivePipeline(const DriveKinematics& kinematics, const SlewRates& linearRates,
                             const SlewRates& strafeRates, const SlewRates& angularRates)
    : m_kinematics(kinematics),
      m_linearSlew(linearRates),
      m_strafeSlew(strafeRates),
      m_angularSlew(angularRates) {}

DriveCommand DrivePipeline::update(const ChannelRequest& linear, const ChannelRequest& angular,
                                   double dtSeconds) {
  return updateHolonomic(linear, ChannelRequest{}, angular, dtSeconds);
}

DriveCommand DrivePipeline::updateHolonomic(const ChannelRequest& linear,
                                            const ChannelRequest& strafe,
                                            const ChannelRequest& angular, double dtSeconds) {
  const DriveSpeeds speeds{.forward = m_linearSlew.update(limitSpeed(linear), dtSeconds),
                           .strafe = m_strafeSlew.update(limitSpeed(strafe), dtSeconds),
                           .turn = m_angularSlew.update(limitSpeed(angular), dtSeconds)};
  return toDriveCommand(m_kinematics.wheelSpeeds(speeds), m_kinematics);
}

DriveCommand DrivePipeline::updatePivot(const ChannelRequest& turn, Side lockedSide,
                                        double dtSeconds) {
  return updatePivotAbout(turn, sidePivot(lockedSide), dtSeconds);
}

DriveCommand DrivePipeline::updatePivotAbout(const ChannelRequest& turn, const LocalOffset& pivot,
                                             double dtSeconds) {
  m_linearSlew.reset();
  m_strafeSlew.reset();
  const double turnSpeed = m_angularSlew.update(limitSpeed(turn), dtSeconds);
  return toDriveCommand(m_kinematics.pivotSpeeds(turnSpeed, pivot), m_kinematics);
}

LocalOffset DrivePipeline::sidePivot(Side lockedSide) const {
  double sumX = 0.0;
  double sumY = 0.0;
  std::size_t count = 0;
  for (std::size_t i = 0; i < m_kinematics.wheelCount(); ++i) {
    const WheelSpec& wheel = m_kinematics.wheel(i);
    if ((lockedSide == Side::Left) == (wheel.xInches < 0.0) && wheel.xInches != 0.0) {
      sumX += wheel.xInches;
      sumY += wheel.yInches;
      ++count;
    }
  }
  if (count == 0) {
    return {};
  }
  return {.rightInches = sumX / count, .forwardInches = sumY / count};
}

void DrivePipeline::reset(double linearSpeed, double angularSpeed, double strafeSpeed) {
  m_linearSlew.reset(linearSpeed);
  m_angularSlew.reset(angularSpeed);
  m_strafeSlew.reset(strafeSpeed);
}

void DrivePipeline::setRates(const SlewRates& linearRates, const SlewRates& angularRates) {
  m_linearSlew.setRates(linearRates);
  m_angularSlew.setRates(angularRates);
}

void DrivePipeline::setRates(const SlewRates& linearRates, const SlewRates& strafeRates,
                             const SlewRates& angularRates) {
  setRates(linearRates, angularRates);
  m_strafeSlew.setRates(strafeRates);
}

}  // namespace cardLib
