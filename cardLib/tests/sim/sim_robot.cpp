// sim_robot.cpp
// Why: implements the simulated motion loop.
// Owns: nothing beyond SimRobot's own state.
#include "sim_robot.hpp"

#include "cardLib/algorithms/math/loop_timing.hpp"

namespace cardLib::sim {

SimRobot::SimRobot(const DriveSimConfig& config, const SlewRates& linearRates,
                   const SlewRates& angularRates, const Pose& start)
    : m_drive(config, start),
      m_estimator({.hasImu = true, .effectiveTrackWidthInches = config.trackWidthInches}),
      m_pipeline(linearRates, angularRates) {
  m_estimator.reset(start);
  m_state = m_estimator.state();
}

void SimRobot::tick(const DriveCommand& command) {
  m_drive.apply(command);
  m_drive.advance(LOOP_PERIOD_SECONDS);
  m_state = m_estimator.update(m_drive.takeFrame(LOOP_PERIOD_SECONDS));
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
      tick({});
      return run;
    }
    tick(step.command);
    run.trace.push_back({elapsedMs, m_drive.truePose(), m_drive.linearSpeedInchesPerSecond()});
  }
  run.elapsedMs = maxMs;
  return run;
}

void SimRobot::idle(std::uint32_t durationMs) {
  for (std::uint32_t elapsedMs = 0; elapsedMs < durationMs; elapsedMs += LOOP_PERIOD_MS) {
    tick({});
  }
}

}  // namespace cardLib::sim
