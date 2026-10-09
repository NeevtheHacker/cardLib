// display_pages.hpp
// Why: turns a snapshot of the robot into the text of each brain-screen page (SPEC 11).
// Owns: page layout and formatting only. No robot logic, no screen calls.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "cardLib/chassis/chassis.hpp"
#include "cardLib/chassis/chassis_config.hpp"
#include "cardLib/chassis/motion_task.hpp"
#include "cardLib/chassis/odometry_task.hpp"
#include "cardLib/systems/black_box.hpp"
#include "cardLib/systems/test_modes.hpp"

namespace cardLib {

constexpr std::size_t DISPLAY_LINES = 8;
constexpr std::size_t DISPLAY_LINE_LENGTH = 40;
constexpr std::size_t USER_LABEL_LENGTH = 20;

using DisplayLine = std::array<char, DISPLAY_LINE_LENGTH>;
using DisplayScreen = std::array<DisplayLine, DISPLAY_LINES>;

// One auton in the selector: a name and the function that runs it.
struct AutonEntry {
  const char* name = "";
  void (*run)() = nullptr;
};

enum class Alliance { Red, Blue };

enum class DisplayPage { Auton, AutonOptions, Pose, Diagnostics, Tests, Tuner, User };
constexpr std::size_t DISPLAY_PAGE_COUNT = 7;

// A line set with Display::set(): a label and a number, formatted only by the display task.
struct UserLine {
  std::array<char, USER_LABEL_LENGTH> label = {};
  double value = 0.0;
  bool used = false;
};

// Everything a page might show, copied once per refresh.
struct DisplayData {
  OdometrySnapshot odometry = {};
  MotionStatus motion = {};
  StartupReport startup = {};
  const char* calibrationSource = "";
  Gains gains = {};
  std::size_t tunerField = 0;
  const char* autonName = "";
  std::size_t autonIndex = 0;
  std::size_t autonCount = 0;
  Alliance alliance = Alliance::Red;
  bool mirrored = false;
  TestKind selectedTest = TestKind::PortCheck;
  PortCheckReport portCheck = {};
  SignCheckReport signCheck = {};
  TapeBoxReport tapeBox = {};
  BlackBox::SaveResult blackBox = BlackBox::SaveResult::NotSaved;
  std::array<UserLine, DISPLAY_LINES> userLines = {};
};

// Fills every line of the screen for this page (blank lines are empty strings).
void renderPage(DisplayPage page, const DisplayData& data, DisplayScreen& screen);

// The gains the Tuner page can step, by index.
constexpr std::size_t TUNER_FIELD_COUNT = 7;
const char* tunerFieldName(std::size_t field);
double* tunerFieldValue(Gains& gains, std::size_t field);

const char* testName(TestKind kind);

}  // namespace cardLib
