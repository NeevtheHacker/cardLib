# Chassis

`#include "cardLib/api.hpp"` · `class cardLib::Chassis`

The API your auton and driver code calls. One per robot.

## Construction and startup

| Member | Description |
|---|---|
| `Chassis(const ChassisConfig& config, PoseEstimator* estimator = nullptr)` | Stores the config. Touches no device, so it's safe as a global. Without an estimator, uses the built-in arc odometry. |
| `const StartupReport& initialize()` | Call once from `initialize()`, robot still. Validates ports and devices, calibrates the IMU (a few seconds), starts the odometry and motion tasks. |
| `const StartupReport& calibrate()` | Re-runs IMU calibration and the drift check. Robot must be still. |
| `bool setCalibration(const Calibration& calibration)` | Replaces calibration values. Only before `initialize()`; returns `false` after. |
| `const StartupReport& startupReport() const` | What startup validation found. |

## Pose

| Member | Description |
|---|---|
| `bool setPose(double xInches, double yInches, double headingDegrees)` | Sets where the robot is. Applied by the odometry task within one loop; `false` if that timed out. |
| `Pose getPose()` | Current `{xInches, yInches, headingDegrees}`. Heading is continuous (720 after two spins). |
| `RobotState getState()` | Pose plus forward and sideways velocity (in/s), angular velocity (°/s, clockwise +), health and the slip warnings. |
| `OdometryHealth odomHealth()` | `Ok`, `Degraded` (on a fallback sensor) or `Faulted`. |

## Pose resets

New in 0.2. Each returns a [`ResetResult`](options-results.md#resetresult). Anything but `Applied` leaves the pose exactly as it was. An applied reset prints one terminal line saying what changed and by how much.

| Member | Description |
|---|---|
| `ResetResult resetFromWall(size_t sensorIndex, Wall wall, const WallResetOptions& = {})` | Sets one field coordinate (x for `Wall::Left`/`Right`, y for `Back`/`Front`) from distance sensor `sensorIndex` (its place in `SensorConfig::distanceSensors`). Only when the robot is stopped, the sensor is within 15° of square to that wall, the reading is in range and the change is under 4 in. Heading and the other coordinate never change. |
| `ResetResult resetFromGps(const GpsResetOptions& = {})` | Sets x and y, and the heading if `.resetHeading = true`, from the GPS. Only when the robot is stopped and the GPS error estimate is under the limit (1.0 in). |

Walls are where `ChassisConfig::field` says (default: a 144 in field with the origin at its center, the same origin as the GPS). Distance math: wall distance = `(gain·reading + offset)·cos(angle off square)`, plus the sensor's position on the robot.

!!! warning "Check the GPS frame on the field"
    cardLib does the GPS mount offset and field rotation itself and assumes the GPS reports +x right, +y forward and heading clockwise. Before trusting `resetFromGps`, put the robot at two taped points facing two headings and check that the pose matches.

## Motions

All block until done and return a [`MotionResult`](options-results.md#motionresult), unless `options.async` is set (then they return `Started` at once).

| Member | Description |
|---|---|
| `move(double distanceInches, const MotionOptions& = {})` | Straight relative move, holding the starting heading. Negative = backward. |
| `turn(double headingDegrees, const MotionOptions& = {})` | Turn in place to a field heading. |
| `turnToPoint(double xInches, double yInches, const MotionOptions& = {})` | Turn in place to face a point (`forwards = false` faces away). |
| `pivotTurn(double headingDegrees, Side lockedSide, const MotionOptions& = {})` | Turn about one locked side (`Side::Left` or `Side::Right`). |
| `moveToPoint(double xInches, double yInches, const MotionOptions& = {})` | Drive to a field point, steering toward it. |
| `moveToPose(double xInches, double yInches, double headingDegrees, const MotionOptions& = {})` | Drive to a point, then turn to a heading. |
| `strafe(double distanceInches, const MotionOptions& = {})` | Slide sideways (positive = right), holding the heading. `Unsupported` on a tank. New in 0.2. |
| `pivotTurn(double headingDegrees, const LocalOffset& pivotPoint, const MotionOptions& = {})` | Turn about a robot-local point `{rightInches, forwardInches}`, e.g. a wheel. A tank can only use points with `forwardInches = 0`; others are `Unsupported`. New in 0.2. |

On a drive that can strafe, `moveToPoint` translates straight to the point holding its heading (or `options.headingDegrees`), and `moveToPose` translates and turns at the same time. `options.tankStyle = true` uses the tank versions. See [Holonomic drives](../tutorials/10-holonomic.md).

## Motion sync

These act on the most recent motion.

| Member | Description |
|---|---|
| `bool waitUntil(double progress)` | Waits until `progress` inches (moves) or degrees (turns). `false` if the motion ended first. |
| `MotionResult waitUntilDone()` | Waits for the end and returns the real result. |
| `bool isMoving()` | `true` while a motion runs. |
| `void cancel()` | Stops the motion; it ends as `Cancelled`. |

## Driver control

| Member | Description |
|---|---|
| `void tank(double leftStick, double rightStick)` | −127 to 127 per side. Cancels a running motion. |
| `void arcade(double forwardStick, double turnStick)` | −127 to 127. Cancels a running motion. |
| `void holonomic(double forward, double strafe, double turn, const HolonomicOptions& = {})` | −127 to 127 each, on any drive. Uses the drive mode, or `{.fieldCentric = true/false}` for this call. Cancels a running motion. New in 0.2. |
| `void setDriveMode(DriveMode mode)` / `DriveMode driveMode() const` | `RobotCentric` (default) or `FieldCentric`. Field-centric falls back to robot-centric if the IMU faults. |
| `void resetFieldHeading()` | Makes the way the robot faces now the field-centric "forward". |
| `bool fieldCentricIgnored() const` | `true` once field-centric was asked for on a drive that can't strafe. |
| `void driveMillivolts(int32_t left, int32_t right)` | Raw voltages, no curves: every left module gets `left`, every right module `right`. For tests and calibration. |
| `void driveMillivolts(std::span<const int32_t> wheelMillivolts)` | Raw voltage per wheel module, in the drive's module order; missing entries get 0. |
| `void setDriverBrake(BrakeMode mode)` / `BrakeMode driverBrake() const` | Brake mode when the sticks are centered. |

## Settings

| Member | Description |
|---|---|
| `void setAllowDegraded(bool allow)` | Let motions run on degraded odometry. Off by default. |
| `void setGains(const Gains& gains)` / `Gains gains()` | Change tuning; takes effect at the next motion loop. |
| `const ChassisConfig& config() const` | The config the chassis was built with. |
| `const DriveConfig& drive() const` | The drive as wheel modules (the tank shorthand turned into two). |
| `const DriveKinematics& driveKinematics() const` | Each wheel's row and the drive's top speeds, built in `initialize()`. |

## For tools and the display

`odometrySnapshot()`, `motionStatus()`, `motionHistory()`, `telemetry()` expose internal state for the display, terminal log, black box and test modes. You won't normally call them.

## StartupReport

| Field | Meaning |
|---|---|
| `validated` | `initialize()` has run |
| `state` | Whether motions may run |
| `portIssues`, `portIssueCount` | Missing or wrong devices, by port |
| `noDrive` | No drive modules: none listed, or `{.left, .right}` without `geometry`. A `SensorFault` |
| `modulesWithoutMotors` | Wheel modules listed with no motor group. Any is a `SensorFault` |
| `noHeadingSource` | No IMU and not enough tracking wheels for heading |
| `imuCalibrationFailed`, `imuOrientationMismatch`, `imuDetectedOrientation` | IMU problems |
| `imuDriftDegrees`, `imuDriftTooHigh` | Drift over the 1 s still check |
| `batteryLow`, `batteryPercent` | Warning only; doesn't block motions |
