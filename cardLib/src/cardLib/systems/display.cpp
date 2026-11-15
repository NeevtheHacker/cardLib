// display.cpp
// Why: implements the display task: buttons, page changes, brain redraws (~10 Hz), and one
//      controller line per tick.
// Owns: nothing beyond Display's own state.
#include "cardLib/systems/display.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "liblvgl/llemu.h"
#include "pros/misc.h"

namespace cardLib {

namespace {

constexpr std::uint32_t BRAIN_EVERY_TICKS = 2;  // 50 ms tick, brain at ~10 Hz
constexpr double TUNER_STEP_FRACTION = 0.10;
constexpr double TUNER_MIN_STEP = 0.001;
constexpr const char* STARTUP_RUMBLE = "---";

template <std::size_t N>
void copyText(std::array<char, N>& destination, const char* source) {
  std::strncpy(destination.data(), source != nullptr ? source : "", N - 1);
  destination[N - 1] = '\0';
}

DisplayPage nextPage(DisplayPage page) {
  const auto next = (static_cast<std::size_t>(page) + 1) % DISPLAY_PAGE_COUNT;
  return static_cast<DisplayPage>(next);
}

}  // namespace

void Display::start() {
  if (m_task != nullptr) {
    return;
  }
  pros::c::lcd_initialize();
  m_task = pros::c::task_create(&Display::taskEntry, this, PRIORITY, TASK_STACK_DEPTH_DEFAULT,
                                "cardLib display");
}

void Display::taskEntry(void* self) { static_cast<Display*>(self)->run(); }

void Display::run() {
  std::uint32_t wakeTimeMs = pros::millis();
  while (true) {
    ++m_tick;
    handleButtons();
    alertOnStartupProblem();
    if (m_tick % BRAIN_EVERY_TICKS == 0) {
      drawBrain();
    }
    updateController(m_tick % 2 == 1);
    pros::Task::delay_until(&wakeTimeMs, TICK_MS);
  }
}

// --- called from user tasks ---------------------------------------------------------------------

void Display::setAutons(std::span<const AutonEntry> autons) {
  m_autons = autons;
  m_autonIndex.store(0);
}

const AutonEntry* Display::selectedAuton() const {
  const std::size_t index = m_autonIndex.load();
  return index < m_autons.size() ? &m_autons[index] : nullptr;
}

void Display::runSelectedAuton() const {
  const AutonEntry* auton = selectedAuton();
  if (auton != nullptr && auton->run != nullptr) {
    auton->run();
  }
}

void Display::set(std::size_t line, const char* label, double value) {
  if (line >= DISPLAY_LINES) {
    return;
  }
  m_mutex.take();
  copyText(m_userLines[line].label, label);
  m_userLines[line].value = value;
  m_userLines[line].used = true;
  m_mutex.give();
}

void Display::setControllerLine(std::size_t line, const char* text) {
  if (line >= CONTROLLER_LINES) {
    return;
  }
  m_mutex.take();
  copyText(m_controllerLines[line], text);
  m_mutex.give();
}

void Display::rumble(const char* pattern) {
  m_mutex.take();
  copyText(m_rumble, pattern);
  m_mutex.give();
}

// --- display task -------------------------------------------------------------------------------

// Center: next page (the Tuner first walks through its fields). Left/right: the page's actions.
void Display::handleButtons() {
  const std::uint8_t buttons = pros::c::lcd_read_buttons();
  const std::uint8_t pressed = buttons & static_cast<std::uint8_t>(~m_lastButtons);
  m_lastButtons = buttons;
  if ((pressed & LCD_BTN_CENTER) != 0) {
    if (m_page == DisplayPage::Tuner && m_tunerField + 1 < TUNER_FIELD_COUNT) {
      ++m_tunerField;
    } else {
      m_page = nextPage(m_page);
      m_tunerField = 0;
    }
  }
  if ((pressed & LCD_BTN_LEFT) != 0) {
    pageAction(true);
  }
  if ((pressed & LCD_BTN_RIGHT) != 0) {
    pageAction(false);
  }
}

void Display::pageAction(bool left) {
  const std::size_t autonCount = m_autons.size();
  switch (m_page) {
    case DisplayPage::Auton:
      if (autonCount > 0) {
        const std::size_t step = left ? autonCount - 1 : 1;
        m_autonIndex.store((m_autonIndex.load() + step) % autonCount);
      }
      break;
    case DisplayPage::AutonOptions:
      if (left) {
        m_alliance.store(m_alliance.load() == Alliance::Red ? Alliance::Blue : Alliance::Red);
      } else {
        m_mirrored.store(!m_mirrored.load());
      }
      break;
    case DisplayPage::Diagnostics:
      if (!left) {
        m_blackBox.saveToSd();
      }
      break;
    case DisplayPage::Tests:
      if (left) {
        m_selectedTest =
            static_cast<TestKind>((static_cast<int>(m_selectedTest) + 1) % TEST_KIND_COUNT);
      } else {
        m_testModes.request(m_selectedTest);
      }
      break;
    case DisplayPage::Tuner:
      stepTuner(!left);
      break;
    case DisplayPage::Pose:
    case DisplayPage::User:
      break;
  }
}

void Display::stepTuner(bool increase) {
  Gains gains = m_chassis.gains();
  double* value = tunerFieldValue(gains, m_tunerField);
  if (value == nullptr) {
    return;
  }
  const double step = std::max(std::abs(*value) * TUNER_STEP_FRACTION, TUNER_MIN_STEP);
  *value = std::max(0.0, *value + (increase ? step : -step));
  m_chassis.setGains(gains);
}

// A failed startup check jumps to Diagnostics and rumbles the controller, once.
void Display::alertOnStartupProblem() {
  const StartupReport& startup = m_chassis.startupReport();
  if (m_alerted || !startup.validated || startup.state == ReadyState::Ready) {
    return;
  }
  m_alerted = true;
  m_page = DisplayPage::Diagnostics;
  rumble(STARTUP_RUMBLE);
}

DisplayData Display::gatherData() {
  DisplayData data;
  data.odometry = m_chassis.odometrySnapshot();
  data.motion = m_chassis.motionStatus();
  data.startup = m_chassis.startupReport();
  data.fieldCentricIgnored = m_chassis.fieldCentricIgnored();
  data.calibrationSource = m_chassis.config().calibration.source;
  data.gains = m_chassis.gains();
  data.tunerField = m_tunerField;
  const AutonEntry* auton = selectedAuton();
  data.autonName = auton != nullptr ? auton->name : "";
  data.autonIndex = m_autonIndex.load();
  data.autonCount = m_autons.size();
  data.alliance = m_alliance.load();
  data.mirrored = m_mirrored.load();
  data.selectedTest = m_selectedTest;
  data.portCheck = m_testModes.portCheckReport();
  data.signCheck = m_testModes.signCheckReport();
  data.tapeBox = m_testModes.tapeBoxReport();
  data.blackBox = m_blackBox.lastResult();
  m_mutex.take();
  data.userLines = m_userLines;
  m_mutex.give();
  return data;
}

// Redraws only the lines that changed since the last draw.
void Display::drawBrain() {
  DisplayScreen screen;
  renderPage(m_page, gatherData(), screen);
  for (std::size_t line = 0; line < DISPLAY_LINES; ++line) {
    if (std::strcmp(screen[line].data(), m_drawn[line].data()) != 0) {
      pros::c::lcd_set_text(static_cast<std::int16_t>(line), screen[line].data());
      m_drawn[line] = screen[line];
    }
  }
}

// One controller packet per tick: a pending rumble first, otherwise the next changed line.
// Two controllers alternate ticks.
void Display::updateController(bool partner) {
  const pros::controller_id_e_t id =
      partner ? pros::E_CONTROLLER_PARTNER : pros::E_CONTROLLER_MASTER;
  if (pros::c::controller_is_connected(id) != 1) {
    return;
  }
  m_mutex.take();
  const std::array<char, RUMBLE_LENGTH> rumble = m_rumble;
  if (!partner) {
    m_rumble[0] = '\0';
  }
  const std::array<ControllerLine, CONTROLLER_LINES> wanted = m_controllerLines;
  m_mutex.give();

  if (!partner && rumble[0] != '\0') {
    pros::c::controller_rumble(id, rumble.data());
    return;
  }
  ControllerState& state = partner ? m_partner : m_master;
  for (std::size_t tried = 0; tried < CONTROLLER_LINES; ++tried) {
    const std::size_t line = state.nextLine;
    state.nextLine = (state.nextLine + 1) % CONTROLLER_LINES;
    if (std::strcmp(wanted[line].data(), state.sent[line].data()) != 0) {
      pros::c::controller_set_text(id, static_cast<std::uint8_t>(line), 0, wanted[line].data());
      state.sent[line] = wanted[line];
      return;
    }
  }
}

}  // namespace cardLib
