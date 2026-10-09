// display_pages.cpp
// Why: the text of each page. Runs only in the display task, never in a 10 ms loop.
// Owns: nothing; every function fills the screen it is given.
#include "cardLib/systems/display_pages.hpp"

#include <cstdio>

namespace cardLib {

namespace {

template <typename... Args>
void setLine(DisplayLine& line, const char* format, Args... args) {
  std::snprintf(line.data(), line.size(), format, args...);
}

const char* healthName(OdometryHealth health) {
  switch (health) {
    case OdometryHealth::Ok:
      return "OK";
    case OdometryHealth::Degraded:
      return "DEGRADED";
    case OdometryHealth::Faulted:
      return "FAULTED";
  }
  return "?";
}

const char* readyName(ReadyState state) {
  switch (state) {
    case ReadyState::Ready:
      return "READY";
    case ReadyState::NotCalibrated:
      return "NOT CALIBRATED";
    case ReadyState::SensorFault:
      return "SENSOR FAULT";
  }
  return "?";
}

const char* problemName(PortProblem problem) {
  switch (problem) {
    case PortProblem::InvalidPort:
      return "bad port";
    case PortProblem::DuplicatePort:
      return "port used twice";
    case PortProblem::Missing:
      return "missing";
    case PortProblem::WrongDevice:
      return "wrong device";
    case PortProblem::WrongAdiMode:
      return "wrong 3-wire mode";
  }
  return "?";
}

const char* orientationName(std::optional<ImuOrientation> orientation) {
  if (!orientation.has_value()) {
    return "unknown";
  }
  constexpr const char* NAMES[] = {"flat", "upside down", "X up", "X down", "Y up", "Y down"};
  return NAMES[static_cast<int>(*orientation)];
}

const char* motionName(MotionKind kind) {
  constexpr const char* NAMES[] = {"none",        "move",          "turn",       "turnToPoint",
                                   "pivotTurn",   "moveToPoint",   "moveToPose"};
  return NAMES[static_cast<int>(kind)];
}

const char* saveName(BlackBox::SaveResult result) {
  switch (result) {
    case BlackBox::SaveResult::NotSaved:
      return "-";
    case BlackBox::SaveResult::Saved:
      return "saved";
    case BlackBox::SaveResult::Busy:
      return "busy (moving)";
    case BlackBox::SaveResult::NoCard:
      return "no SD card";
    case BlackBox::SaveResult::OpenFailed:
      return "can't open file";
  }
  return "?";
}

void renderAuton(const DisplayData& data, DisplayScreen& screen) {
  setLine(screen[0], "Auton   L/R choose, C page");
  setLine(screen[1], "> %s", data.autonCount > 0 ? data.autonName : "(no autons set)");
  setLine(screen[2], "%u of %u", static_cast<unsigned>(data.autonIndex + 1),
          static_cast<unsigned>(data.autonCount));
  setLine(screen[3], "%s%s", data.alliance == Alliance::Red ? "RED" : "BLUE",
          data.mirrored ? "  mirrored" : "");
}

void renderAutonOptions(const DisplayData& data, DisplayScreen& screen) {
  setLine(screen[0], "Auton options");
  setLine(screen[1], "L: alliance  %s", data.alliance == Alliance::Red ? "RED" : "BLUE");
  setLine(screen[2], "R: mirror    %s", data.mirrored ? "on" : "off");
}

void renderPose(const DisplayData& data, DisplayScreen& screen) {
  const RobotState& state = data.odometry.state;
  setLine(screen[0], "x %7.2f   y %7.2f", state.pose.xInches, state.pose.yInches);
  setLine(screen[1], "heading %7.2f deg", wrapDegrees(state.pose.headingDegrees));
  setLine(screen[2], "odom %s%s", healthName(state.health),
          state.wheelHeadingSuspect ? "  wheel slip?" : "");
  setLine(screen[3], "CAL: %s", data.calibrationSource);
  setLine(screen[4], "v %6.1f in/s  w %6.1f deg/s", state.linearVelocityInchesPerSecond,
          state.angularVelocityDegreesPerSecond);
  if (data.motion.moving) {
    setLine(screen[5], "%s  %.1f -> %.1f", motionName(data.motion.kind), data.motion.measured,
            data.motion.target);
  } else {
    setLine(screen[5], "%s", data.motion.driverControl ? "driver control" : "idle");
  }
  const OdometrySnapshot& odometry = data.odometry;
  setLine(screen[6], "faults:%s%s%s%s%s%s", odometry.verticalWheelFaulted[0] ? " V1" : "",
          odometry.verticalWheelFaulted[1] ? " V2" : "",
          odometry.horizontalWheelFaulted ? " H" : "", odometry.leftDriveFaulted ? " L" : "",
          odometry.rightDriveFaulted ? " R" : "", odometry.imuFaulted ? " IMU" : "");
  const SensorRejections& rejects = odometry.rejections;
  setLine(screen[7], "rejects V%lu/%lu H%lu L%lu R%lu IMU%lu",
          static_cast<unsigned long>(rejects.verticalWheels[0]),
          static_cast<unsigned long>(rejects.verticalWheels[1]),
          static_cast<unsigned long>(rejects.horizontalWheel),
          static_cast<unsigned long>(rejects.leftDrive),
          static_cast<unsigned long>(rejects.rightDrive), static_cast<unsigned long>(rejects.imu));
}

// Startup problems, most important first, as many as fit above the black-box line.
void renderDiagnostics(const DisplayData& data, DisplayScreen& screen) {
  const StartupReport& startup = data.startup;
  std::size_t line = 0;
  setLine(screen[line++], "Startup: %s", startup.validated ? readyName(startup.state) : "-");
  const std::size_t lastFree = DISPLAY_LINES - 1;
  for (std::size_t i = 0; i < startup.portIssueCount && line < lastFree; ++i) {
    const PortIssue& issue = startup.portIssues[i];
    setLine(screen[line++], "P%u %s: %s", static_cast<unsigned>(issue.port), issue.role,
            problemName(issue.problem));
  }
  if (startup.noHeadingSource && line < lastFree) {
    setLine(screen[line++], "no heading source (IMU/2 wheels)");
  }
  if (startup.imuOrientationMismatch && line < lastFree) {
    setLine(screen[line++], "IMU mounted %s: check config",
            orientationName(startup.imuDetectedOrientation));
  }
  if (startup.imuCalibrationFailed && line < lastFree) {
    setLine(screen[line++], "IMU calibration failed");
  }
  if (line < lastFree) {
    setLine(screen[line++], "IMU drift %.3f deg%s", startup.imuDriftDegrees,
            startup.imuDriftTooHigh ? " TOO HIGH" : "");
  }
  if (line < lastFree) {
    setLine(screen[line++], "battery %.0f%%%s  overruns %lu", startup.batteryPercent,
            startup.batteryLow ? " LOW" : "",
            static_cast<unsigned long>(data.odometry.loopOverruns));
  }
  setLine(screen[lastFree], "R: save black box  %s", saveName(data.blackBox));
}

void renderPortCheck(const PortCheckReport& report, DisplayScreen& screen) {
  if (!report.ran) {
    setLine(screen[2], "wheels OFF the ground, then R");
    return;
  }
  for (std::size_t i = 0; i < report.motorCount && 2 + i < DISPLAY_LINES; ++i) {
    const MotorCheck& motor = report.motors[i];
    setLine(screen[2 + i], "P%u %c %+5.0f rpm %s %s", static_cast<unsigned>(motor.port),
            motor.side == Side::Left ? 'L' : 'R', motor.rpm,
            motor.directionOk ? "dir ok" : "DIR?", motor.speedOk ? "spd ok" : "SPEED?");
  }
}

void renderSignCheck(const SignCheckReport& report, DisplayScreen& screen) {
  if (report.inProgress) {
    setLine(screen[2], "push fwd, push right, turn CW");
    setLine(screen[3], "then R to finish");
    return;
  }
  if (!report.ran) {
    setLine(screen[2], "R to start; all should read +");
    return;
  }
  const SensorTotals& change = report.change;
  setLine(screen[2], "V1 %+6.1f  V2 %+6.1f", change.verticalWheelInches[0],
          change.verticalWheelInches[1]);
  setLine(screen[3], "H  %+6.1f", change.horizontalWheelInches);
  setLine(screen[4], "L  %+6.1f  R  %+6.1f", change.leftDriveInches, change.rightDriveInches);
  setLine(screen[5], "IMU %+6.1f deg", change.imuRotationDegrees);
}

void renderTapeBox(const TapeBoxReport& report, DisplayScreen& screen) {
  if (report.inProgress) {
    setLine(screen[2], "roll the robot, return it to");
    setLine(screen[3], "the taped box, then R");
    return;
  }
  if (!report.ran) {
    setLine(screen[2], "robot in the taped box, then R");
    return;
  }
  setLine(screen[2], "position error %.2f in", report.error.positionInches);
  setLine(screen[3], "heading error  %.2f deg", report.error.headingDegrees);
}

void renderTests(const DisplayData& data, DisplayScreen& screen) {
  setLine(screen[0], "Tests   L next, R run, C page");
  setLine(screen[1], "> %s", testName(data.selectedTest));
  switch (data.selectedTest) {
    case TestKind::PortCheck:
      renderPortCheck(data.portCheck, screen);
      break;
    case TestKind::SignCheck:
      renderSignCheck(data.signCheck, screen);
      break;
    case TestKind::TapeBox:
      renderTapeBox(data.tapeBox, screen);
      break;
  }
}

void renderTuner(const DisplayData& data, DisplayScreen& screen) {
  Gains gains = data.gains;
  const double* value = tunerFieldValue(gains, data.tunerField);
  setLine(screen[0], "Tuner: %s", tunerFieldName(data.tunerField));
  setLine(screen[1], "value %.4f", value != nullptr ? *value : 0.0);
  setLine(screen[2], "L -10%%  R +10%%  C next");
  setLine(screen[3], "target %.2f  meas %.2f", data.motion.target, data.motion.measured);
  setLine(screen[4], "out L %ld  R %ld mV", static_cast<long>(data.motion.leftMillivolts),
          static_cast<long>(data.motion.rightMillivolts));
}

void renderUser(const DisplayData& data, DisplayScreen& screen) {
  for (std::size_t i = 0; i < DISPLAY_LINES; ++i) {
    const UserLine& user = data.userLines[i];
    if (user.used) {
      setLine(screen[i], "%s: %.3f", user.label.data(), user.value);
    }
  }
}

}  // namespace

void renderPage(DisplayPage page, const DisplayData& data, DisplayScreen& screen) {
  for (DisplayLine& line : screen) {
    line[0] = '\0';
  }
  switch (page) {
    case DisplayPage::Auton:
      renderAuton(data, screen);
      break;
    case DisplayPage::AutonOptions:
      renderAutonOptions(data, screen);
      break;
    case DisplayPage::Pose:
      renderPose(data, screen);
      break;
    case DisplayPage::Diagnostics:
      renderDiagnostics(data, screen);
      break;
    case DisplayPage::Tests:
      renderTests(data, screen);
      break;
    case DisplayPage::Tuner:
      renderTuner(data, screen);
      break;
    case DisplayPage::User:
      renderUser(data, screen);
      break;
  }
}

const char* tunerFieldName(std::size_t field) {
  constexpr const char* NAMES[TUNER_FIELD_COUNT] = {
      "move kP", "move kD", "heading kP", "turn kP", "turn kD", "pivot kP", "pivot kD"};
  return field < TUNER_FIELD_COUNT ? NAMES[field] : "?";
}

double* tunerFieldValue(Gains& gains, std::size_t field) {
  switch (field) {
    case 0:
      return &gains.linear.kP;
    case 1:
      return &gains.linear.kD;
    case 2:
      return &gains.heading.kP;
    case 3:
      return &gains.turn.kP;
    case 4:
      return &gains.turn.kD;
    case 5:
      return &gains.pivot.kP;
    case 6:
      return &gains.pivot.kD;
    default:
      return nullptr;
  }
}

const char* testName(TestKind kind) {
  switch (kind) {
    case TestKind::PortCheck:
      return "port + direction check";
    case TestKind::SignCheck:
      return "sensor sign check";
    case TestKind::TapeBox:
      return "tape box test";
  }
  return "?";
}

}  // namespace cardLib
