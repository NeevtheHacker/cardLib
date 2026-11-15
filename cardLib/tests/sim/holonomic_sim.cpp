// holonomic_sim.cpp
// Why: implements the wheel-module drive simulator.
// Owns: nothing beyond HolonomicSimRobot's own state.
#include "holonomic_sim.hpp"

#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"
#include "cardLib/algorithms/math/loop_timing.hpp"

namespace cardLib::sim {

namespace {

ArcOdometryConfig robotOdometry(const HolonomicSimConfig& config) {
  ArcOdometryConfig odometry;
  odometry.verticalWheelCount = config.verticalWheel ? 1 : 0;
  odometry.hasHorizontalWheel = config.horizontalWheel;
  odometry.hasImu = true;
  odometry.driveKinematics = config.kinematics;
  return odometry;
}

}  // namespace

HolonomicSimRobot::HolonomicSimRobot(const HolonomicSimConfig& config, const SlewRates& linearRates,
                                     const SlewRates& strafeRates, const SlewRates& angularRates,
                                     const Pose& start)
    : m_config(config),
      m_estimator(robotOdometry(config)),
      m_pipeline(config.kinematics, linearRates, strafeRates, angularRates),
      m_truePose(start) {
  m_estimator.reset(start);
  m_state = m_estimator.state();
}

Point HolonomicSimRobot::truePointOnRobot(const LocalOffset& local) const {
  return localToField(m_truePose, local);
}

// Each wheel: dv/dt = (v_top * V / 12 - v) / tau, held wheels stop at once. The body moves by
// the least-squares fit of what the wheels rolled; the robot's sensors see that motion.
void HolonomicSimRobot::tick(const DriveCommand& command) {
  const DriveKinematics& kinematics = m_config.kinematics;
  const std::size_t count = kinematics.wheelCount();
  std::array<double, MAX_WHEEL_MODULES> travel = {};
  std::array<bool, MAX_WHEEL_MODULES> usable = {};
  for (std::size_t i = 0; i < count; ++i) {
    const WheelCommand wheel = command.wheels[i];
    const double target = kinematics.wheel(i).topSpeedInchesPerSecond * wheel.millivolts /
                          static_cast<double>(MAX_MILLIVOLTS);
    double& speed = m_wheelSpeeds[i];
    speed = wheel.mode == SideMode::Held
                ? 0.0
                : speed + (target - speed) * LOOP_PERIOD_SECONDS / m_config.timeConstantSeconds;
    travel[i] = speed * LOOP_PERIOD_SECONDS;
    usable[i] = true;
  }
  const BodyMotion body = kinematics.bodyMotion(std::span<const double>(travel.data(), count),
                                                std::span<const bool>(usable.data(), count));

  const double midHeading = degreesToRadians(m_truePose.headingDegrees) + body.headingRadians / 2.0;
  m_truePose.xInches +=
      body.rightInches * std::cos(midHeading) + body.forwardInches * std::sin(midHeading);
  m_truePose.yInches +=
      -body.rightInches * std::sin(midHeading) + body.forwardInches * std::cos(midHeading);
  m_truePose.headingDegrees += radiansToDegrees(body.headingRadians);

  SensorFrame frame;
  frame.dtSeconds = LOOP_PERIOD_SECONDS;
  frame.verticalWheels[0] = {body.forwardInches, m_config.verticalWheel};
  frame.horizontalWheel = {body.rightInches, m_config.horizontalWheel};
  frame.imuReadOk = true;
  frame.imuRotationDegrees = m_truePose.headingDegrees;
  frame.driveWheelCount = count;
  for (std::size_t i = 0; i < count; ++i) {
    frame.driveWheels[i] = {travel[i], true};
  }
  m_state = m_estimator.update(frame);
}

SimRun HolonomicSimRobot::run(MotionAlgorithm& motion, std::uint32_t maxMs) {
  SimRun run;
  motion.start(m_state);
  for (std::uint32_t elapsedMs = 0; elapsedMs <= maxMs; elapsedMs += LOOP_PERIOD_MS) {
    const MotionStep step = motion.step(m_state, {.elapsedMs = elapsedMs}, m_pipeline);
    if (step.exit.has_value()) {
      run.result = step.exit->result;
      run.brake = step.exit->brake;
      run.finished = true;
      run.elapsedMs = elapsedMs;
      if (run.brake) {
        m_pipeline.reset();
      }
      tick(toDriveCommand({.count = m_config.kinematics.wheelCount()}, m_config.kinematics));
      return run;
    }
    tick(step.command);
    run.trace.push_back({elapsedMs, m_truePose, 0.0, m_state.health});
  }
  run.elapsedMs = maxMs;
  return run;
}

void HolonomicSimRobot::idle(std::uint32_t durationMs) {
  const DriveCommand stopped =
      toDriveCommand({.count = m_config.kinematics.wheelCount()}, m_config.kinematics);
  for (std::uint32_t elapsedMs = 0; elapsedMs < durationMs; elapsedMs += LOOP_PERIOD_MS) {
    tick(stopped);
  }
}

}  // namespace cardLib::sim
