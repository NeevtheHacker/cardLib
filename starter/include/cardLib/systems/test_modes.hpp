// test_modes.hpp
// Why: the Tests page checks: port and direction, sensor signs, tape box (SPEC 6, 13, 17).
// Owns: the requested test and the last results. Drives only through the chassis, and reads
//       sensors only through the odometry snapshot.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "cardLib/algorithms/calibration/calibration_math.hpp"
#include "cardLib/algorithms/math/pose.hpp"
#include "cardLib/chassis/chassis.hpp"
#include "cardLib/chassis/odometry_task.hpp"
#include "cardLib/hardware/motor_group.hpp"

namespace cardLib {

enum class TestKind { PortCheck, SignCheck, TapeBox };
constexpr std::size_t TEST_KIND_COUNT = 3;

// One drive motor at 10 % with its side's wheels off the ground.
struct MotorCheck {
  std::uint8_t port = 0;
  Side side = Side::Left;
  bool answering = false;
  double rpm = 0.0;
  bool directionOk = false;  // forward voltage turned it forward
  bool speedOk = false;      // near 10 % of its cartridge: a wrong cartridge shows up here
};

struct PortCheckReport {
  bool ran = false;
  std::array<MotorCheck, 2 * MAX_MOTORS_PER_GROUP> motors = {};
  std::size_t motorCount = 0;
};

// Changes over a hand push: forward, then right, then a clockwise turn. Every one should be
// positive.
struct SignCheckReport {
  bool ran = false;
  bool inProgress = false;
  SensorTotals change = {};
};

struct TapeBoxReport {
  bool ran = false;
  bool inProgress = false;
  TapeBoxError error = {};
};

class TestModes {
 public:
  explicit TestModes(Chassis& chassis) : m_chassis(chassis) {}

  // The display asks; user code runs it from opcontrol with runPending(), since tests move the
  // robot and the screen must never contain robot logic.
  void request(TestKind kind) { m_requested.store(static_cast<int>(kind)); }
  void runPending();

  // Wheels off the ground: each side at 10 % for 1 s, every motor's speed read (blocking).
  PortCheckReport runPortCheck();

  // Two presses each: start, then finish after moving the robot by hand.
  void toggleSignCheck();
  void toggleTapeBox();

  // Copies of the last results, safe to call from the display task.
  PortCheckReport portCheckReport();
  SignCheckReport signCheckReport();
  TapeBoxReport tapeBoxReport();

 private:
  static constexpr int NO_REQUEST = -1;

  void checkSide(MotorGroup* group, Side side, PortCheckReport& report);

  Chassis& m_chassis;
  std::atomic<int> m_requested{NO_REQUEST};
  pros::Mutex m_resultsMutex;
  PortCheckReport m_portCheck;
  SignCheckReport m_signCheck;
  SensorTotals m_signStart = {};
  TapeBoxReport m_tapeBox;
  Pose m_tapeBoxStart = {};
};

}  // namespace cardLib
