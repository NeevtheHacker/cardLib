// test_modes.cpp
// Why: implements the on-robot test modes.
// Owns: nothing beyond TestModes' own state.
#include "cardLib/systems/test_modes.hpp"

#include <cmath>

#include "cardLib/algorithms/control/drive_mixer.hpp"

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

PortCheckReport TestModes::runPortCheck() {
  PortCheckReport report;
  report.ran = true;
  checkSide(m_chassis.config().drive.left, Side::Left, report);
  checkSide(m_chassis.config().drive.right, Side::Right, report);
  m_resultsMutex.take();
  m_portCheck = report;
  m_resultsMutex.give();
  return report;
}

void TestModes::checkSide(MotorGroup* group, Side side, PortCheckReport& report) {
  if (group == nullptr) {
    return;
  }
  const auto millivolts = static_cast<std::int32_t>(TEST_FRACTION * MAX_MILLIVOLTS);
  m_chassis.driveMillivolts(side == Side::Left ? millivolts : 0,
                            side == Side::Right ? millivolts : 0);
  pros::delay(SPIN_UP_MS);
  for (std::size_t i = 0; i < group->size(); ++i) {
    MotorCheck& check = report.motors[report.motorCount++];
    check.port = group->motor(i).port;
    check.side = side;
    const std::optional<double> rpm = group->velocityRpm(i);
    check.answering = rpm.has_value();
    check.rpm = rpm.value_or(0.0);
    const double expectedRpm = TEST_FRACTION * cartridgeRpm(group->motor(i).cartridge);
    check.directionOk = check.rpm > 0.0;
    check.speedOk =
        std::abs(check.rpm - expectedRpm) <= SPEED_TOLERANCE_FRACTION * expectedRpm;
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
