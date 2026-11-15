// sim_robot.cpp
// Why: implements the simulated motion loop.
// Owns: nothing beyond SimRobot's own state.
#include "sim_robot.hpp"

#include "cardLib/algorithms/localization/wheel_distance.hpp"
#include "cardLib/algorithms/math/loop_timing.hpp"

namespace cardLib::sim {

SimRobot::SimRobot(const DriveSimConfig& config, const SlewRates& linearRates,
                   const SlewRates& angularRates, const Pose& start)
    : m_drive(config, start),
      m_estimator({.hasImu = true, .effectiveTrackWidthInches = config.trackWidthInches}),
      m_leftCheck(maxWheelSpeedInchesPerSecond(config.wheelDiameterInches, config.wheelRpm)),
      m_rightCheck(maxWheelSpeedInchesPerSecond(config.wheelDiameterInches, config.wheelRpm)),
      m_pipeline(linearRates, angularRates) {
  m_estimator.reset(start);
  m_leftCheck.rebaseline(m_drive.leftReportedInches());
  m_rightCheck.rebaseline(m_drive.rightReportedInches());
  m_state = m_estimator.state();
}

void SimRobot::failLeftEncoderReads(std::uint32_t atElapsedMs, std::uint32_t loops) {
  m_failStartMs = atElapsedMs;
  m_failLoops = loops;
}

// The same reads the odometry task makes: each encoder through its plausibility check, and a
// perfect IMU.
SensorFrame SimRobot::readSensors(std::uint32_t elapsedMs) {
  const bool leftFails =
      elapsedMs >= m_failStartMs && elapsedMs < m_failStartMs + m_failLoops * LOOP_PERIOD_MS;
  const std::optional<double> left =
      leftFails ? std::nullopt : std::optional<double>(m_drive.leftReportedInches());

  SensorFrame frame;
  frame.dtSeconds = LOOP_PERIOD_SECONDS;
  frame.leftDrive = m_leftCheck.measure(left, LOOP_PERIOD_SECONDS);
  frame.rightDrive = m_rightCheck.measure(m_drive.rightReportedInches(), LOOP_PERIOD_SECONDS);
  frame.imuReadOk = true;
  frame.imuRotationDegrees = m_drive.truePose().headingDegrees;
  return frame;
}

void SimRobot::tick(const DriveCommand& command, std::uint32_t elapsedMs) {
  m_drive.apply(command);
  m_drive.advance(LOOP_PERIOD_SECONDS);
  m_state = m_estimator.update(readSensors(elapsedMs));
}

SimRun SimRobot::run(MotionAlgorithm& motion, std::uint32_t maxMs) {
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
      tick({}, elapsedMs);
      m_failLoops = 0;
      return run;
    }
    tick(step.command, elapsedMs);
    run.trace.push_back(
        {elapsedMs, m_drive.truePose(), m_drive.linearSpeedInchesPerSecond(), m_state.health});
  }
  m_failLoops = 0;
  run.elapsedMs = maxMs;
  return run;
}

void SimRobot::idle(std::uint32_t durationMs) {
  for (std::uint32_t elapsedMs = 0; elapsedMs < durationMs; elapsedMs += LOOP_PERIOD_MS) {
    tick({}, 0);
  }
}

}  // namespace cardLib::sim
