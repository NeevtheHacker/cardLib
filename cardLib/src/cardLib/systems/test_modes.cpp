// test_modes.cpp
// Why: implements the on-robot test modes.
// Owns: nothing beyond TestModes' own state.
#include "cardLib/systems/test_modes.hpp"

#include <array>
#include <cmath>
#include <cstdio>

#include "cardLib/algorithms/control/drive_mixer.hpp"
#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

namespace {
constexpr double TEST_FRACTION = 0.10;          // 10 % voltage
constexpr std::uint32_t SPIN_UP_MS = 800;       // read speed after this
constexpr std::uint32_t SPIN_TOTAL_MS = 1000;
constexpr double SPEED_TOLERANCE_FRACTION = 0.5;  // within +-50 % of expected (tune)
constexpr std::uint32_t SETTLE_MS = 300;
}  // namespace

void TestModes::runPending() {
  const int requested = m_requested.exchange(NO_REQUEST);
  switch (requested) {
    case static_cast<int>(TestKind::PortCheck):
      runPortCheck();
      break;
    case static_cast<int>(TestKind::SignCheck):
      toggleSignCheck();
      break;
    case static_cast<int>(TestKind::TapeBox):
      toggleTapeBox();
      break;
    default:
      break;
  }
}

const char* pushDirectionName(double rollAngleDegrees) {
  constexpr const char* NAMES[] = {"backward",      "backward-left", "left",  "forward-left",
                                   "forward",       "forward-right", "right", "backward-right"};
  constexpr double STEP_DEGREES = 45.0;
  const double steps = std::round(wrapDegrees(rollAngleDegrees) / STEP_DEGREES);
  const int index = static_cast<int>(steps) + 4;
  if (std::abs(wrapDegrees(rollAngleDegrees) - steps * STEP_DEGREES) > 1.0 || index < 0 ||
      index > 7) {
    return "at its roll angle";
  }
  return NAMES[index];
}

PortCheckReport TestModes::runPortCheck() {
  PortCheckReport report;
  report.ran = true;
  for (std::size_t module = 0; module < m_chassis.drive().moduleCount; ++module) {
    checkModule(module, report);
  }
  m_resultsMutex.take();
  m_portCheck = report;
  m_resultsMutex.give();
  return report;
}

// Spins one module alone: a positive command should turn every motor in it forward, which
// pushes the robot the way its roll angle says (SPEC 20.3).
void TestModes::checkModule(std::size_t module, PortCheckReport& report) {
  const DriveModule& drive = m_chassis.drive().modules[module];
  MotorGroup* const group = drive.motors;
  if (group == nullptr) {
    return;
  }
  std::printf("module %u (x %.1f, y %.1f, roll %+.0f deg): should push the robot %s\n",
              static_cast<unsigned>(module + 1), drive.xInches, drive.yInches,
              drive.rollAngleDegrees, pushDirectionName(drive.rollAngleDegrees));
  std::array<std::int32_t, MAX_WHEEL_MODULES> millivolts = {};
  millivolts[module] = static_cast<std::int32_t>(TEST_FRACTION * MAX_MILLIVOLTS);
  m_chassis.driveMillivolts(millivolts);
  pros::delay(SPIN_UP_MS);
  for (std::size_t i = 0; i < group->size() && report.motorCount < report.motors.size(); ++i) {
    MotorCheck& check = report.motors[report.motorCount++];
    check.port = group->motor(i).port;
    check.module = module;
    check.rollAngleDegrees = drive.rollAngleDegrees;
    check.side = drive.xInches < 0.0 ? Side::Left : Side::Right;
    const std::optional<double> rpm = group->velocityRpm(i);
    check.answering = rpm.has_value();
    check.rpm = rpm.value_or(0.0);
    const double expectedRpm = TEST_FRACTION * motorRpm(group->motor(i));
    check.directionOk = check.rpm > 0.0;
    check.speedOk =
        std::abs(check.rpm - expectedRpm) <= SPEED_TOLERANCE_FRACTION * expectedRpm;
    std::printf("  P%u %+.0f rpm  %s  %s\n", static_cast<unsigned>(check.port), check.rpm,
                check.directionOk ? "direction ok" : "WRONG DIRECTION",
                check.speedOk ? "speed ok" : "SPEED? (cartridge)");
  }
  pros::delay(SPIN_TOTAL_MS - SPIN_UP_MS);
  m_chassis.driveMillivolts(0, 0);
  pros::delay(SETTLE_MS);
}

void TestModes::toggleSignCheck() {
  const SensorTotals now = m_chassis.odometrySnapshot().totals;
  if (!m_signCheck.inProgress) {
    m_signStart = now;
    m_resultsMutex.take();
    m_signCheck = {.ran = false, .inProgress = true};
    m_resultsMutex.give();
    return;
  }
  SignCheckReport report{.ran = true, .inProgress = false};
  for (std::size_t i = 0; i < MAX_VERTICAL_WHEELS; ++i) {
    report.change.verticalWheelInches[i] =
        now.verticalWheelInches[i] - m_signStart.verticalWheelInches[i];
  }
  report.change.horizontalWheelInches =
      now.horizontalWheelInches - m_signStart.horizontalWheelInches;
  report.change.leftDriveInches = now.leftDriveInches - m_signStart.leftDriveInches;
  report.change.rightDriveInches = now.rightDriveInches - m_signStart.rightDriveInches;
  report.change.imuRotationDegrees = now.imuRotationDegrees - m_signStart.imuRotationDegrees;
  m_resultsMutex.take();
  m_signCheck = report;
  m_resultsMutex.give();
}

void TestModes::toggleTapeBox() {
  const Pose now = m_chassis.getPose();
  if (!m_tapeBox.inProgress) {
    m_tapeBoxStart = now;
    m_resultsMutex.take();
    m_tapeBox = {.ran = false, .inProgress = true};
    m_resultsMutex.give();
    return;
  }
  m_resultsMutex.take();
  m_tapeBox = {.ran = true, .inProgress = false, .error = tapeBoxError(m_tapeBoxStart, now)};
  m_resultsMutex.give();
}

PortCheckReport TestModes::portCheckReport() {
  m_resultsMutex.take();
  const PortCheckReport copy = m_portCheck;
  m_resultsMutex.give();
  return copy;
}

SignCheckReport TestModes::signCheckReport() {
  m_resultsMutex.take();
  const SignCheckReport copy = m_signCheck;
  m_resultsMutex.give();
  return copy;
}

TapeBoxReport TestModes::tapeBoxReport() {
  m_resultsMutex.take();
  const TapeBoxReport copy = m_tapeBox;
  m_resultsMutex.give();
  return copy;
}

}  // namespace cardLib
