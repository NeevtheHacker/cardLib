// move_to_pose.cpp
// Why: implements the two-phase moveToPose() from SPEC section 8.
// Owns: nothing beyond MoveToPose's own state.
#include "cardLib/algorithms/motion/move_to_pose.hpp"

#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

namespace {

// The drive phase can't settle on its own windows; only the hand-off gate (or a failure)
// ends it. A large window of 0 also lets the stall check run at any distance.
MoveToPointSettings driveOnlySettings(MoveToPointSettings settings) {
  settings.exitWindows.smallError = 0.0;
  settings.exitWindows.largeError = 0.0;
  return settings;
}

MotionOptions driveOptions(MotionOptions options) {
  options.minSpeed = 0.0;
  options.earlyExit = 0.0;
  options.direction = TurnDirection::Shortest;
  return options;
}

}  // namespace

MoveToPose::MoveToPose(const MoveToPoseSettings& settings)
    : m_settings(settings),
      m_drive(driveOnlySettings(settings.drive)),
      m_turn(settings.turn) {}

void MoveToPose::configure(const Pose& target, const MotionOptions& options) {
  m_target = target;
  m_options = options;
}

void MoveToPose::setSettings(const MoveToPoseSettings& settings) {
  m_settings = settings;
  m_drive.setSettings(driveOnlySettings(settings.drive));
  m_turn.setSettings(settings.turn);
}

void MoveToPose::start(const RobotState& state) {
  m_phase = Phase::Drive;
  m_driveProgressInches = 0.0;
  m_drive.configure({m_target.xInches, m_target.yInches}, driveOptions(m_options));
  m_drive.start(state);
}

MotionStep MoveToPose::step(const RobotState& state, const MotionTick& tick,
                            DrivePipeline& pipeline) {
  if (m_phase == Phase::Drive) {
    return stepDrive(state, tick, pipeline);
  }
  MotionStep result = m_turn.step(state, tick, pipeline);
  result.progress = m_driveProgressInches;
  return result;
}

MotionStep MoveToPose::stepDrive(const RobotState& state, const MotionTick& tick,
                                 DrivePipeline& pipeline) {
  if (readyToTurn(state)) {
    return beginTurn(state, tick, pipeline);
  }
  MotionStep result = m_drive.step(state, tick, pipeline);
  m_driveProgressInches = result.progress;
  return result;
}

// Hand-off gate: close enough and slow enough that the turn is a true turn in place.
bool MoveToPose::readyToTurn(const RobotState& state) const {
  return m_drive.distanceToTargetInches(state.pose) < m_settings.handoffRadiusInches &&
         std::abs(state.linearVelocityInchesPerSecond) < m_settings.handoffSpeedInchesPerSecond;
}

MotionStep MoveToPose::beginTurn(const RobotState& state, const MotionTick& tick,
                                 DrivePipeline& pipeline) {
  m_phase = Phase::Turn;
  m_turn.configureHeading(m_target.headingDegrees, m_options);
  m_turn.start(state);

  const double headingError = turnErrorDegrees(m_target.headingDegrees, state.pose.headingDegrees);
  if (std::abs(headingError) < m_settings.turn.exitWindows.smallError) {
    MotionStep skipped;
    skipped.exit = ExitDecision{MotionResult::Settled, true};
    skipped.progress = m_driveProgressInches;
    return skipped;
  }
  MotionStep result = m_turn.step(state, tick, pipeline);
  result.progress = m_driveProgressInches;
  return result;
}

}  // namespace cardLib
