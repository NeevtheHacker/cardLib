// motion_task.cpp
// Why: the motion loop: take the command slot, step the active motion through the shared
//      pipeline, write the drive, post the result (SPEC sections 2, 8 and 9).
// Owns: nothing beyond MotionTask's own state.
#include "cardLib/chassis/motion_task.hpp"

#include <cmath>
#include <cstdlib>
#include <span>

#include "cardLib/algorithms/control/drive_mixer.hpp"
#include "cardLib/algorithms/control/exit_conditions.hpp"
#include "cardLib/algorithms/math/loop_timing.hpp"

namespace cardLib {

namespace {

constexpr std::uint32_t WAIT_POLL_MS = 10;
constexpr double MICROSECONDS_PER_SECOND = 1e6;

MoveSettings moveSettings(const Gains& gains, const Limits& limits) {
  return {gains.linear, gains.heading, limits.moveExits, limits.useCurrentForStall};
}

TurnSettings turnSettings(const Gains& gains, const Limits& limits) {
  return {gains.turn, limits.turnExits, limits.useCurrentForStall};
}

PivotTurnSettings pivotSettings(const Gains& gains, const Limits& limits) {
  return {gains.pivot, limits.pivotExits, limits.useCurrentForStall};
}

MoveToPointSettings moveToPointSettings(const Gains& gains, const Limits& limits) {
  return {gains.linear, gains.heading, limits.moveExits, limits.useCurrentForStall,
          limits.settleRadiusInches};
}

MoveToPoseSettings moveToPoseSettings(const Gains& gains, const Limits& limits) {
  return {moveToPointSettings(gains, limits), turnSettings(gains, limits),
          limits.handoffRadiusInches, limits.handoffSpeedInchesPerSecond};
}

// What the black box records as a motion's target.
Pose recordedTarget(const MotionCommand& command, const Pose& startPose) {
  if (command.kind == MotionKind::Move) {
    return {0.0, command.distanceInches, startPose.headingDegrees};
  }
  return command.target;
}

}  // namespace

MotionTask::MotionTask(const ChassisConfig& config, OdometryTask& odometry,
                       MotionHistory& history, TelemetryBuffer& telemetry)
    : m_config(config),
      m_odometry(odometry),
      m_history(history),
      m_telemetry(telemetry),
      m_driverBrake(config.limits.driverBrake),
      m_move(moveSettings(config.gains, config.limits)),
      m_turn(turnSettings(config.gains, config.limits)),
      m_pivotTurn(pivotSettings(config.gains, config.limits)),
      m_moveToPoint(moveToPointSettings(config.gains, config.limits)),
      m_moveToPose(moveToPoseSettings(config.gains, config.limits)),
      m_pipeline(config.gains.linearSlew, config.gains.angularSlew) {}

void MotionTask::start() {
  if (m_task != nullptr) {
    return;
  }
  m_task = pros::c::task_create(&MotionTask::taskEntry, this, PRIORITY, TASK_STACK_DEPTH_DEFAULT,
                                "cardLib motion");
}

void MotionTask::taskEntry(void* self) { static_cast<MotionTask*>(self)->run(); }

void MotionTask::run() {
  std::uint32_t wakeTimeMs = pros::millis();
  std::uint64_t lastMicros = pros::micros();
  while (true) {
    const std::uint64_t nowMicros = pros::micros();
    const double dtSeconds = (nowMicros - lastMicros) / MICROSECONDS_PER_SECOND;
    lastMicros = nowMicros;

    const Intake intake = takeIntake();
    const OdometrySnapshot odometry = m_odometry.snapshot();
    handleIntake(intake, odometry);
    if (m_active != nullptr) {
      stepActive(odometry, dtSeconds, intake);
    } else if (intake.driverMode) {
      applyDriver(intake.driver, intake.driverBrake);
    }
    pros::Task::delay_until(&wakeTimeMs, LOOP_PERIOD_MS);
  }
}

// --- user-task side -------------------------------------------------------------------------

std::uint32_t MotionTask::post(const MotionCommand& command) {
  m_mutex.take();
  if (m_postedId != m_takenId) {
    recordResult(m_postedId, MotionResult::Cancelled);
  }
  m_slot = command;
  const std::uint32_t id = ++m_postedId;
  m_cancelRequested = false;
  m_driverMode = false;
  m_mutex.give();
  return id;
}

void MotionTask::drive(const DriverCommand& command) {
  m_mutex.take();
  if (m_postedId != m_takenId) {
    recordResult(m_postedId, MotionResult::Cancelled);
    m_takenId = m_postedId;
  }
  m_driverMode = true;
  m_driverCommand = command;
  m_mutex.give();
}

void MotionTask::cancel() {
  m_mutex.take();
  if (m_postedId != m_takenId) {
    recordResult(m_postedId, MotionResult::Cancelled);
    m_takenId = m_postedId;
  }
  m_cancelRequested = true;
  m_mutex.give();
}

MotionResult MotionTask::waitUntilDone(std::uint32_t id) {
  while (true) {
    m_mutex.take();
    const std::optional<MotionResult> result = findResult(id);
    if (!result.has_value()) {
      m_waiter = pros::c::task_get_current();
    }
    m_mutex.give();
    if (result.has_value()) {
      return *result;
    }
    pros::c::task_notify_take(true, WAIT_POLL_MS);
  }
}

bool MotionTask::waitUntil(std::uint32_t id, double progress) {
  while (true) {
    m_mutex.take();
    const bool ended = findResult(id).has_value();
    const bool reached =
        m_status.activeId == id && m_status.moving && m_status.progress >= progress;
    m_mutex.give();
    if (reached) {
      return true;
    }
    if (ended) {
      return false;
    }
    pros::delay(WAIT_POLL_MS);
  }
}

MotionStatus MotionTask::status() {
  m_mutex.take();
  const MotionStatus copy = m_status;
  m_mutex.give();
  return copy;
}

void MotionTask::setReadyState(ReadyState state) {
  m_mutex.take();
  m_ready = state;
  m_mutex.give();
}

void MotionTask::setGains(const Gains& gains) {
  m_mutex.take();
  m_pendingGains = gains;
  m_mutex.give();
}

void MotionTask::setDriverBrake(BrakeMode mode) {
  m_mutex.take();
  m_driverBrake = mode;
  m_mutex.give();
}

// Called with m_mutex held.
void MotionTask::recordResult(std::uint32_t id, MotionResult result) {
  m_results[m_nextResult] = {id, result};
  m_nextResult = (m_nextResult + 1) % RESULT_HISTORY;
}

// Called with m_mutex held. Empty while the motion is waiting or running.
std::optional<MotionResult> MotionTask::findResult(std::uint32_t id) const {
  if (id == 0 || id > m_postedId) {
    return MotionResult::Cancelled;
  }
  for (const FinishedMotion& finished : m_results) {
    if (finished.id == id) {
      return finished.result;
    }
  }
  const bool waitingOrRunning = id > m_takenId || (m_status.moving && m_status.activeId == id);
  if (waitingOrRunning) {
    return std::nullopt;
  }
  return MotionResult::Cancelled;  // older than the result history
}

// --- motion-task side -----------------------------------------------------------------------

MotionTask::Intake MotionTask::takeIntake() {
  Intake intake;
  m_mutex.take();
  if (m_postedId != m_takenId) {
    intake.command = m_slot;
    intake.commandId = m_postedId;
    m_takenId = m_postedId;
    // Running from the moment it's taken, so a waiter never sees it as neither waiting,
    // running nor finished.
    m_status = {.moving = true, .kind = m_slot.kind, .activeId = m_postedId};
  }
  intake.cancel = m_cancelRequested;
  m_cancelRequested = false;
  intake.driverMode = m_driverMode;
  intake.driver = m_driverCommand;
  intake.driverBrake = m_driverBrake;
  intake.ready = m_ready;
  intake.gains = m_pendingGains;
  m_pendingGains.reset();
  m_mutex.give();
  return intake;
}

void MotionTask::handleIntake(const Intake& intake, const OdometrySnapshot& odometry) {
  if (intake.gains.has_value()) {
    applyGains(*intake.gains);
  }
  const bool replaced = intake.command.has_value() || intake.driverMode;
  if (m_active != nullptr && replaced) {
    finishActive(MotionResult::Cancelled, false, odometry.state.pose);
  }
  if (intake.command.has_value()) {
    startMotion(*intake.command, intake.commandId, odometry.state);
  }
}

void MotionTask::startMotion(const MotionCommand& command, std::uint32_t id,
                             const RobotState& state) {
  m_active = configureMotion(command);
  if (m_active == nullptr) {
    m_mutex.take();
    recordResult(id, MotionResult::Cancelled);
    m_status.moving = false;
    m_mutex.give();
    return;
  }
  m_activeCommand = command;
  m_activeId = id;
  m_startTimeMs = pros::millis();
  m_startPose = state.pose;
  m_active->start(state);
}

MotionAlgorithm* MotionTask::configureMotion(const MotionCommand& command) {
  const Point point{command.target.xInches, command.target.yInches};
  switch (command.kind) {
    case MotionKind::Move:
      m_move.configure(command.distanceInches, command.options);
      return &m_move;
    case MotionKind::Turn:
      m_turn.configureHeading(command.target.headingDegrees, command.options);
      return &m_turn;
    case MotionKind::TurnToPoint:
      m_turn.configurePoint(point, command.options);
      return &m_turn;
    case MotionKind::PivotTurn:
      m_pivotTurn.configure(command.target.headingDegrees, command.lockedSide, command.options);
      return &m_pivotTurn;
    case MotionKind::MoveToPoint:
      m_moveToPoint.configure(point, command.options);
      return &m_moveToPoint;
    case MotionKind::MoveToPose:
      m_moveToPose.configure(command.target, command.options);
      return &m_moveToPose;
    case MotionKind::None:
      break;
  }
  return nullptr;
}

void MotionTask::stepActive(const OdometrySnapshot& odometry, double dtSeconds,
                            const Intake& intake) {
  const MotionTick tick{
      .dtSeconds = dtSeconds,
      .elapsedMs = pros::millis() - m_startTimeMs,
      .calibrated = intake.ready != ReadyState::NotCalibrated,
      .cancelled = intake.cancel,
      .driveFault = intake.ready == ReadyState::SensorFault || odometry.driveMotorMissing,
      .currentConfirmsStall = currentStallConfirmed(m_lastCommand),
  };
  const MotionStep step = m_active->step(odometry.state, tick, m_pipeline);
  if (step.exit.has_value()) {
    finishActive(step.exit->result, step.exit->brake, odometry.state.pose);
    return;
  }
  applyCommand(step.command);
  publishStatus(step);
}

void MotionTask::finishActive(MotionResult result, bool brake, const Pose& endPose) {
  stopDrive(brake ? m_config.limits.motionBrake : BrakeMode::Coast);
  if (brake) {
    m_pipeline.reset();
  }
  const std::uint32_t nowMs = pros::millis();
  m_history.add({m_activeCommand.kind, recordedTarget(m_activeCommand, m_startPose), m_startPose,
                 endPose, result, m_startTimeMs, nowMs - m_startTimeMs});

  m_mutex.take();
  recordResult(m_activeId, result);
  if (m_status.activeId == m_activeId) {
    m_status.moving = false;  // not when a newer motion has already been taken
  }
  const pros::task_t waiter = m_waiter;
  m_waiter = nullptr;
  m_mutex.give();

  if (waiter != nullptr) {
    pros::c::task_notify(waiter);
  }
  m_active = nullptr;
}

void MotionTask::publishStatus(const MotionStep& step) {
  m_mutex.take();
  m_status.progress = step.progress;
  m_status.target = step.target;
  m_status.measured = step.measured;
  m_status.leftMillivolts = step.command.leftMillivolts;
  m_status.rightMillivolts = step.command.rightMillivolts;
  m_mutex.give();

  if ((m_config.limits.telemetryChannels & TELEMETRY_MOTION) != 0) {
    const double averageMillivolts =
        (step.command.leftMillivolts + step.command.rightMillivolts) / 2.0;
    m_telemetry.push(
        {pros::millis(), TelemetryChannel::Motion, step.target, step.measured, averageMillivolts});
  }
}

void MotionTask::applyCommand(const DriveCommand& command) {
  MotorGroup* const left = m_config.drive.left;
  MotorGroup* const right = m_config.drive.right;
  if (left != nullptr) {
    if (command.leftMode == SideMode::Held) {
      left->stop(BrakeMode::Hold);
    } else {
      left->moveMillivolts(command.leftMillivolts);
    }
  }
  if (right != nullptr) {
    if (command.rightMode == SideMode::Held) {
      right->stop(BrakeMode::Hold);
    } else {
      right->moveMillivolts(command.rightMillivolts);
    }
  }
  m_lastCommand = command;
}

void MotionTask::applyDriver(const DriverCommand& command, BrakeMode brake) {
  m_pipeline.reset();
  if (command.leftMillivolts == 0 && command.rightMillivolts == 0) {
    stopDrive(brake);
  } else {
    applyCommand({.leftMillivolts = command.leftMillivolts,
                  .rightMillivolts = command.rightMillivolts});
  }
  m_mutex.take();
  m_status.driverControl = true;
  m_status.leftMillivolts = command.leftMillivolts;
  m_status.rightMillivolts = command.rightMillivolts;
  m_mutex.give();
}

void MotionTask::stopDrive(BrakeMode mode) {
  if (m_config.drive.left != nullptr) {
    m_config.drive.left->stop(mode);
  }
  if (m_config.drive.right != nullptr) {
    m_config.drive.right->stop(mode);
  }
  m_lastCommand = {};
}

void MotionTask::applyGains(const Gains& gains) {
  const Limits& limits = m_config.limits;
  m_move.setSettings(moveSettings(gains, limits));
  m_turn.setSettings(turnSettings(gains, limits));
  m_pivotTurn.setSettings(pivotSettings(gains, limits));
  m_moveToPoint.setSettings(moveToPointSettings(gains, limits));
  m_moveToPose.setSettings(moveToPoseSettings(gains, limits));
  m_pipeline.setRates(gains.linearSlew, gains.angularSlew);
}

// SPEC section 9 optional current check: half the drive motors over the threshold while the
// commanded output is above a minimum. Reads current per motor, never the vector version.
bool MotionTask::currentStallConfirmed(const DriveCommand& command) const {
  const Limits& limits = m_config.limits;
  if (!limits.useCurrentForStall) {
    return false;
  }
  std::array<double, 2 * MAX_MOTORS_PER_GROUP> currents = {};
  std::size_t count = 0;
  for (const MotorGroup* group : {m_config.drive.left, m_config.drive.right}) {
    for (std::size_t i = 0; group != nullptr && i < group->size(); ++i) {
      if (const std::optional<double> amps = group->currentAmps(i)) {
        currents[count++] = *amps;
      }
    }
  }
  const double averageMillivolts =
      (std::abs(command.leftMillivolts) + std::abs(command.rightMillivolts)) / 2.0;
  const double commandedSpeed = averageMillivolts / MAX_MILLIVOLTS * MAX_SPEED;
  return currentConfirmsStall(std::span<const double>(currents.data(), count),
                              limits.stallCurrentAmps, commandedSpeed,
                              limits.stallMinCommandSpeed);
}

}  // namespace cardLib
