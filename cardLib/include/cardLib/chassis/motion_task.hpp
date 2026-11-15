// motion_task.hpp
// Why: runs the active motion (or driver control) every 10 ms.
// Owns: the drive motors (only writer), the command slot, the motion objects and the shared
//       drive pipeline.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "cardLib/algorithms/control/drive_pipeline.hpp"
#include "cardLib/algorithms/interfaces/motion_algorithm.hpp"
#include "cardLib/algorithms/motion/motion_result.hpp"
#include "cardLib/algorithms/motion/holonomic_move_to_point.hpp"
#include "cardLib/algorithms/motion/holonomic_move_to_pose.hpp"
#include "cardLib/algorithms/motion/move.hpp"
#include "cardLib/algorithms/motion/move_to_point.hpp"
#include "cardLib/algorithms/motion/move_to_pose.hpp"
#include "cardLib/algorithms/motion/pivot_turn.hpp"
#include "cardLib/algorithms/motion/strafe.hpp"
#include "cardLib/algorithms/motion/turn.hpp"
#include "cardLib/chassis/chassis_config.hpp"
#include "cardLib/chassis/motion_command.hpp"
#include "cardLib/chassis/motion_history.hpp"
#include "cardLib/chassis/odometry_task.hpp"
#include "cardLib/chassis/telemetry_buffer.hpp"
#include "pros/rtos.hpp"

namespace cardLib {

// What the motion task is doing, for waits, the screen and the tuner page.
struct MotionStatus {
  bool moving = false;
  bool driverControl = false;
  MotionKind kind = MotionKind::None;
  std::uint32_t activeId = 0;
  double progress = 0.0;
  double target = 0.0;
  double measured = 0.0;
  std::int32_t leftMillivolts = 0;
  std::int32_t rightMillivolts = 0;
};

class MotionTask {
 public:
  static constexpr std::uint32_t PRIORITY = 9;

  MotionTask(const ChassisConfig& config, OdometryTask& odometry, MotionHistory& history,
             TelemetryBuffer& telemetry);

  void start();

  // Puts a command in the slot and returns its ID. A motion still running is cancelled.
  std::uint32_t post(const MotionCommand& command);

  // Switches to driver control (cancelling any motion) with these voltages.
  void drive(const DriverCommand& command);

  void cancel();

  // Blocks until the motion with this ID has a result.
  MotionResult waitUntilDone(std::uint32_t id);

  // Blocks until that motion's progress reaches the value. False if it ended first.
  bool waitUntil(std::uint32_t id, double progress);

  MotionStatus status();

  // The resolved drive and its kinematics. Call before start(), once the motor groups exist.
  void setDrive(const DriveConfig& drive, const DriveKinematics& kinematics);

  void setReadyState(ReadyState state);
  void setGains(const Gains& gains);
  void setDriverBrake(BrakeMode mode);

 private:
  static constexpr std::size_t RESULT_HISTORY = 8;

  struct FinishedMotion {
    std::uint32_t id = 0;
    MotionResult result = MotionResult::Settled;
  };

  // What the task picks up from the slot at the start of a loop.
  struct Intake {
    std::optional<MotionCommand> command;
    std::uint32_t commandId = 0;
    bool cancel = false;
    bool driverMode = false;
    DriverCommand driver = {};
    BrakeMode driverBrake = BrakeMode::Coast;
    ReadyState ready = ReadyState::NotCalibrated;
    std::optional<Gains> gains;
  };

  static void taskEntry(void* self);
  void run();
  Intake takeIntake();
  void handleIntake(const Intake& intake, const OdometrySnapshot& odometry);
  void startMotion(const MotionCommand& command, std::uint32_t id, const RobotState& state);
  MotionAlgorithm* configureMotion(const MotionCommand& command);
  bool runsHolonomic(const MotionCommand& command) const;
  void stepActive(const OdometrySnapshot& odometry, double dtSeconds, const Intake& intake);
  void finishActive(MotionResult result, bool brake, const Pose& endPose);
  void applyCommand(const DriveCommand& command);
  WheelCommand wheelCommandFor(const DriveCommand& command, std::size_t module) const;
  void applyDriver(const DriverCommand& command, BrakeMode brake);
  void stopDrive(BrakeMode mode);
  void applyGains(const Gains& gains);
  bool currentStallConfirmed(const DriveCommand& command) const;
  void recordResult(std::uint32_t id, MotionResult result);
  std::optional<MotionResult> findResult(std::uint32_t id) const;
  void publishStatus(const MotionStep& step);

  const ChassisConfig& m_config;
  DriveConfig m_drive;  // resolved: always wheel modules
  OdometryTask& m_odometry;
  MotionHistory& m_history;
  TelemetryBuffer& m_telemetry;

  // Shared with user tasks, guarded by m_mutex.
  pros::Mutex m_mutex;
  MotionCommand m_slot = {};
  std::uint32_t m_postedId = 0;
  std::uint32_t m_takenId = 0;
  bool m_cancelRequested = false;
  bool m_driverMode = false;
  DriverCommand m_driverCommand = {};
  BrakeMode m_driverBrake;
  ReadyState m_ready = ReadyState::NotCalibrated;
  std::optional<Gains> m_pendingGains;
  std::array<FinishedMotion, RESULT_HISTORY> m_results = {};
  std::size_t m_nextResult = 0;
  MotionStatus m_status = {};
  pros::task_t m_waiter = nullptr;

  // Used only by the motion task.
  Move m_move;
  Turn m_turn;
  PivotTurn m_pivotTurn;
  MoveToPoint m_moveToPoint;
  MoveToPose m_moveToPose;
  Strafe m_strafe;
  HolonomicMoveToPoint m_holonomicMoveToPoint;
  HolonomicMoveToPose m_holonomicMoveToPose;
  DrivePipeline m_pipeline;
  MotionAlgorithm* m_active = nullptr;
  MotionCommand m_activeCommand = {};
  std::uint32_t m_activeId = 0;
  std::uint32_t m_startTimeMs = 0;
  Pose m_startPose = {};
  DriveCommand m_lastCommand = {};
  pros::task_t m_task = nullptr;
};

}  // namespace cardLib
