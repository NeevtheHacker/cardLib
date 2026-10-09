// display.hpp
// Why: one task owns the brain screen and the controller screen; everything else just hands
//      it values (SPEC section 11).
// Owns: the display task (priority 3, 50 ms tick), the auton selection and the user lines.
//       The only cardLib code that calls LLEMU or writes to the controller.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

#include "cardLib/chassis/chassis.hpp"
#include "cardLib/systems/black_box.hpp"
#include "cardLib/systems/display_pages.hpp"
#include "cardLib/systems/test_modes.hpp"
#include "pros/rtos.hpp"

namespace cardLib {

class Display {
 public:
  static constexpr std::uint32_t PRIORITY = 3;
  static constexpr std::uint32_t TICK_MS = 50;
  static constexpr std::size_t CONTROLLER_LINES = 3;
  static constexpr std::size_t CONTROLLER_LINE_LENGTH = 16;  // 15 characters fit
  static constexpr std::size_t RUMBLE_LENGTH = 9;            // 8 symbols fit

  Display(Chassis& chassis, TestModes& testModes, BlackBox& blackBox)
      : m_chassis(chassis), m_testModes(testModes), m_blackBox(blackBox) {}

  // Initializes the LLEMU screen and starts the task. Call once from initialize().
  void start();

  // Auton selector. The list must outlive the display (a const global array).
  void setAutons(std::span<const AutonEntry> autons);
  const AutonEntry* selectedAuton() const;
  Alliance alliance() const { return m_alliance.load(); }
  bool mirrored() const { return m_mirrored.load(); }

  // Runs the selected auton, if any. Call from autonomous().
  void runSelectedAuton() const;

  // Shows "label: value" on the User page. Copies both; formatting happens in the display task.
  void set(std::size_t line, const char* label, double value);

  // Controller lines 0 to 2. At most one line is sent per tick, and only if it changed.
  void setControllerLine(std::size_t line, const char* text);
  void rumble(const char* pattern);

 private:
  using ControllerLine = std::array<char, CONTROLLER_LINE_LENGTH>;

  // What has been sent to one controller, so only changes go out.
  struct ControllerState {
    std::array<ControllerLine, CONTROLLER_LINES> sent = {};
    std::size_t nextLine = 0;
  };

  static void taskEntry(void* self);
  void run();
  void handleButtons();
  void pageAction(bool left);
  void stepTuner(bool increase);
  void alertOnStartupProblem();
  DisplayData gatherData();
  void drawBrain();
  void updateController(bool partner);

  Chassis& m_chassis;
  TestModes& m_testModes;
  BlackBox& m_blackBox;

  pros::Mutex m_mutex;  // guards the user-set values below
  std::array<UserLine, DISPLAY_LINES> m_userLines = {};
  std::array<ControllerLine, CONTROLLER_LINES> m_controllerLines = {};
  std::array<char, RUMBLE_LENGTH> m_rumble = {};

  std::span<const AutonEntry> m_autons;
  std::atomic<std::size_t> m_autonIndex{0};
  std::atomic<Alliance> m_alliance{Alliance::Red};
  std::atomic<bool> m_mirrored{false};

  // Used only by the display task.
  DisplayPage m_page = DisplayPage::Auton;
  std::size_t m_tunerField = 0;
  TestKind m_selectedTest = TestKind::PortCheck;
  std::uint8_t m_lastButtons = 0;
  bool m_alerted = false;
  DisplayScreen m_drawn = {};
  ControllerState m_master;
  ControllerState m_partner;
  std::uint32_t m_tick = 0;
  pros::task_t m_task = nullptr;
};

}  // namespace cardLib
