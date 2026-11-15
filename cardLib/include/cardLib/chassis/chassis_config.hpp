// chassis_config.hpp
// Why: the four kinds of robot values (SPEC section 3) kept in four structs, plus the devices.
// Owns: Geometry, Calibration, Gains, Limits, DriveModule, DriveConfig, SensorConfig,
//       ChassisConfig.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "cardLib/algorithms/control/drive_curve.hpp"
#include "cardLib/algorithms/control/drive_kinematics.hpp"
#include "cardLib/algorithms/control/exit_conditions.hpp"
#include "cardLib/algorithms/control/pid_controller.hpp"
#include "cardLib/algorithms/control/slew_limiter.hpp"
#include "cardLib/algorithms/localization/arc_odometry.hpp"
#include "cardLib/algorithms/localization/pose_reset.hpp"
#include "cardLib/algorithms/localization/sensor_frame.hpp"
#include "cardLib/hardware/hardware_config.hpp"
#include "cardLib/hardware/motor_group.hpp"

namespace cardLib {

// One driven wheel as the chassis stores it (SPEC 20.1). Robot code builds these with tank(),
// asterisk() or customDrive() from drive_presets.hpp, whose WheelModule can't leave out the
// diameter, gearing or roll angle.
struct DriveModule {
  MotorGroup* motors = nullptr;
  double wheelDiameterInches = 0.0;
  double gearRatio = 1.0;  // G_ext: wheel turns per motor output turn, external gears only
  double xInches = 0.0;    // wheel contact point, right of the robot's center
  double yInches = 0.0;    // and forward of it
  double rollAngleDegrees = 0.0;  // the way a positive command pushes, clockwise from forward
};

// The drivetrain. Either wheel modules from a preset, or the tank shorthand: left and right
// motor groups described by Geometry. With modules, Geometry's drive fields are not used.
struct DriveConfig {
  MotorGroup* left = nullptr;
  MotorGroup* right = nullptr;
  std::array<DriveModule, MAX_WHEEL_MODULES> modules = {};
  std::size_t moduleCount = 0;
};

// Every positional sensor, by pointer to its config (declared once in the robot's devices file).
struct SensorConfig {
  const TrackingWheelConfig* verticalWheel1 = nullptr;
  const TrackingWheelConfig* verticalWheel2 = nullptr;
  const TrackingWheelConfig* horizontalWheel = nullptr;
  const ImuConfig* imu = nullptr;
  std::array<const DistanceSensorConfig*, MAX_DISTANCE_SENSORS> distanceSensors = {};
  const GpsConfig* gps = nullptr;
};

// Physical facts. Change when the robot is rebuilt. The drive fields describe the tank
// shorthand (DriveConfig left/right); a drive built from modules carries its own.
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
  PidGains strafe = {.kP = 10.0};  // strafe distance; holonomic moveToPoint uses linear (tune)
  SlewRates strafeSlew = {.accelPerSecond = 500.0, .decelPerSecond = 1500.0};  // (tune)
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
  FieldWalls field = {};  // where resetFromWall() finds each wall (SPEC 21.2)
};

// The default estimator's settings, from the geometry, sensor configs and calibration.
ArcOdometryConfig makeArcOdometryConfig(const ChassisConfig& config);

// The drive as wheel modules: the config's own, or the tank shorthand turned into a left and a
// right module from Geometry. Reads only numbers and pointers, so it is safe in constructors.
DriveConfig resolveDrive(const ChassisConfig& config);

// True when every module rolls straight forward: a tank, however its wheels are listed.
bool isTankDrive(const DriveConfig& drive);

// Mean x of the right-side modules minus mean x of the left-side ones; 0 if a side is empty.
double driveTrackWidthInches(const DriveConfig& drive);

// A module's motor output rpm, from its first motor's type and cartridge (600 if it has none).
// Reads the motor group, so call it only once the groups exist (from initialize()).
double moduleMotorRpm(const DriveModule& module);

// The module's top surface speed, pi * D * motorRpm * G_ext / 60 (SPEC 20.5).
double moduleTopSpeedInchesPerSecond(const DriveModule& module);

// The kinematics of a resolved drive, from each module's own motors (SPEC 20.7).
DriveKinematics makeDriveKinematics(const DriveConfig& drive);

// The same rows with every top speed set to 1: enough for forward kinematics, and reads no motor.
DriveKinematics driveGeometryKinematics(const DriveConfig& drive);

// The fastest any drive wheel's surface can move: the plausibility check's speed limit.
double maxDriveSpeedInchesPerSecond(const ChassisConfig& config);

}  // namespace cardLib
