// chassis_setup.cpp
// Why: geometry, gains and limits for this robot, and the cardLib::Chassis built from them.
// Owns: the chassis and the services around it (display, terminal log, tests, calibration).
#include "robot/chassis_setup.hpp"

#include "autons/auton_list.hpp"
#include "mechanisms/in_out_mechanism.hpp"
#include "mechanisms/pneumatics.hpp"
#include "robot/calibration_values.hpp"
#include "robot/devices.hpp"

namespace {

constexpr const char* CALIBRATION_FILE = "/usd/cardLib/robot.cal";

cardLib::ChassisConfig makeChassisConfig() {
  cardLib::ChassisConfig config;
  config.drive = {.left = &leftDrive, .right = &rightDrive};
  config.sensors.verticalWheel1 = &verticalTrackingWheel;
  config.sensors.horizontalWheel = &horizontalTrackingWheel;
  config.sensors.imu = &imuConfig;

  // Physical facts: 3.25 in wheels, 36:48 external gearing.
  config.geometry = {.wheelDiameterInches = 3.25, .trackWidthInches = 12.25, .externalRatio = 0.75};
  config.calibration = robotCalibration();

  // Starting gains (tune with the Tuner page, then copy the values here).
  config.gains.linear = {.kP = 10.0, .kD = 0.5};
  config.gains.heading = {.kP = 2.0};
  config.gains.turn = {.kP = 2.0, .kD = 0.2};
  config.gains.pivot = {.kP = 3.0, .kD = 0.2};

  config.limits.defaultTimeoutMs = 3000;
  config.limits.driverBrake = cardLib::BrakeMode::Coast;
  config.limits.throttleCurve = {.deadzone = 3.0, .curveGain = 3.0};
  config.limits.turnCurve = {.deadzone = 3.0, .curveGain = 5.0};
  config.limits.telemetryChannels = 0;  // e.g. cardLib::TELEMETRY_MOTION while tuning
  return config;
}

}  // namespace

cardLib::Chassis chassis(makeChassisConfig());
cardLib::TestModes testModes(chassis);
cardLib::BlackBox blackBox(chassis);
cardLib::Display display(chassis, testModes, blackBox);
cardLib::TerminalLog terminalLog(chassis.telemetry());
cardLib::CalibrationRoutines calibrationRoutines(chassis);

void setupRobot() {
  cardLib::Calibration calibration = chassis.config().calibration;
  if (cardLib::CalibrationFile::load(CALIBRATION_FILE, calibration) ==
      cardLib::CalibrationFile::LoadResult::Loaded) {
    chassis.setCalibration(calibration);
  }

  intakeMotors.initialize();
  clamp.initialize();

  display.setAutons(autonList());
  display.start();
  terminalLog.start();
  chassis.initialize();
}
