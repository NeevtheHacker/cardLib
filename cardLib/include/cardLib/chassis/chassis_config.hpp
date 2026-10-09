// chassis_config.hpp
// Why: the four kinds of robot values (SPEC section 3) kept in four structs, plus the devices.
// Owns: Geometry, Calibration, Gains, Limits, DriveConfig, SensorConfig, ChassisConfig.
#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "cardLib/algorithms/control/drive_curve.hpp"
#include "cardLib/algorithms/control/exit_conditions.hpp"
#include "cardLib/algorithms/control/pid_controller.hpp"
#include "cardLib/algorithms/control/slew_limiter.hpp"
#include "cardLib/algorithms/localization/arc_odometry.hpp"
#include "cardLib/algorithms/localization/sensor_frame.hpp"
#include "cardLib/hardware/hardware_config.hpp"
#include "cardLib/hardware/motor_group.hpp"

namespace cardLib {

struct DriveConfig {
  MotorGroup* left = nullptr;
  MotorGroup* right = nullptr;
};

// Every positional sensor, by pointer to its config (declared once in the robot's devices file).
struct SensorConfig {
  const TrackingWheelConfig* verticalWheel1 = nullptr;
  const TrackingWheelConfig* verticalWheel2 = nullptr;
  const TrackingWheelConfig* horizontalWheel = nullptr;
  const ImuConfig* imu = nullptr;
  std::array<const DistanceSensorConfig*, MAX_DISTANCE_SENSORS> distanceSensors = {};
};

// Physical facts. Change when the robot is rebuilt.
struct Geometry {
  double wheelDiameterInches = 3.25;
  double trackWidthInches = 12.0;
  double externalRatio = 1.0;  // G_ext: wheel turns per motor output turn, external gears only
};

// Measured corrections. Change when calibration is re-run. An empty value means "not measured;
// use the geometry or sensor config".
struct Calibration {
  double imuScale = 1.0;    // k_imu
  double driveScale = 1.0;  // s_drive
  std::optional<double> effectiveTrackWidthInches = std::nullopt;
  std::array<std::optional<double>, MAX_VERTICAL_WHEELS> verticalWheelXInches = {};
  std::array<std::optional<double>, MAX_VERTICAL_WHEELS> verticalWheelDiameterInches = {};
  std::optional<double> horizontalWheelYInches = std::nullopt;
  std::optional<double> horizontalWheelDiameterInches = std::nullopt;
  const char* source = "code";  // shown on screen, e.g. "code" or "SD 10/03"
};

// Tuning.
struct Gains {
  PidGains linear = {.kP = 10.0};   // move / moveToPoint distance (tune)
  PidGains heading = {.kP = 2.0};   // heading hold during moves (tune)
  PidGains turn = {.kP = 2.0};      // turn / turnToPoint / moveToPose final turn (tune)
  PidGains pivot = {.kP = 3.0};     // pivotTurn (tune)
  SlewRates linearSlew = {.accelPerSecond = 500.0, .decelPerSecond = 1500.0};    // (tune)
  SlewRates angularSlew = {.accelPerSecond = 1500.0, .decelPerSecond = 3000.0};  // (tune)
};

// Telemetry channels the terminal prints; 0 prints nothing (a match build).
enum TelemetryChannelBits : std::uint32_t {
  TELEMETRY_MOTION = 1u << 0,  // target, measured, average drive output (mV)
  TELEMETRY_POSE = 1u << 1,    // x, y, heading
};

// Behavior and preferences.
struct Limits {
  std::uint32_t defaultTimeoutMs = 3000;
  BrakeMode motionBrake = BrakeMode::Brake;  // after a settled motion
  BrakeMode driverBrake = BrakeMode::Coast;  // when the sticks are centered
  ExitWindows moveExits = MOVE_EXIT_WINDOWS;
  ExitWindows turnExits = TURN_EXIT_WINDOWS;
  ExitWindows pivotExits = PIVOT_EXIT_WINDOWS;
  double settleRadiusInches = 6.0;            // (tune)
  double handoffRadiusInches = 1.0;           // (tune)
  double handoffSpeedInchesPerSecond = 3.0;   // (tune)
  bool useCurrentForStall = false;
  double stallCurrentAmps = 2.0;              // below the 2.5 A motor limit (tune)
  double stallMinCommandSpeed = 20.0;         // speed units (tune)
  DriveCurve throttleCurve = {};
  DriveCurve turnCurve = {};
  double velocitySmoothing = 0.5;             // EMA alpha (tune)
  double imuDriftLimitDegrees = 0.05;         // over the 1 s still check (tune)
  std::uint32_t imuCalibrationTimeoutMs = 3000;
  double batteryWarningPercent = 30.0;        // (tune)
  std::uint32_t telemetryChannels = 0;
};

struct ChassisConfig {
  DriveConfig drive = {};
  SensorConfig sensors = {};
  Geometry geometry = {};
  Calibration calibration = {};
  Gains gains = {};
  Limits limits = {};
};

// The default estimator's settings, from the geometry, sensor configs and calibration.
ArcOdometryConfig makeArcOdometryConfig(const ChassisConfig& config);

// The chassis' top speed, pi * D * rpm / 60 with rpm = cartridge rpm * G_ext.
double maxDriveSpeedInchesPerSecond(const ChassisConfig& config);

}  // namespace cardLib
