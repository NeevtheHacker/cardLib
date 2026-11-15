# At a glance

Every function and preset a team calls, one line each. Follow a link for the full entry: arguments, defaults and the details. Everything is in `#include "cardLib/api.hpp"`, namespace `cardLib`.

Units everywhere: inches, degrees (0 = +y, clockwise positive), milliseconds, speeds 0 to 127 (sticks −127 to 127). Every motion has a timeout and returns a [`MotionResult`](options-results.md#motionresult).

## Startup

| Call | What it does |
|---|---|
| `Chassis chassis(config)` · [`Chassis(config, estimator)`](chassis.md#construction-and-startup) | Builds the chassis from a `ChassisConfig`. Safe as a global: touches no device. |
| [`chassis.initialize()`](chassis.md#construction-and-startup) | Call once in `initialize()`, robot still: checks ports, calibrates the IMU, starts the tasks. |
| [`chassis.calibrate()`](chassis.md#construction-and-startup) | Re-runs IMU calibration and the drift check. |
| [`chassis.setCalibration(calibration)`](chassis.md#construction-and-startup) | Replaces the measured calibration values. Only before `initialize()`. |
| [`chassis.startupReport()`](chassis.md#startupreport) | What startup found: port problems, IMU problems, battery. |

## Pose

| Call | What it does |
|---|---|
| [`chassis.setPose(x, y, heading)`](chassis.md#pose) | Tells odometry where the robot is. |
| [`chassis.getPose()`](chassis.md#pose) | Where the robot is: `{xInches, yInches, headingDegrees}`. |
| [`chassis.getState()`](chassis.md#pose) | Pose plus velocities, odometry health and the slip warnings. |
| [`chassis.odomHealth()`](chassis.md#pose) | `Ok`, `Degraded` or `Faulted`. |
| [`chassis.resetFromWall(sensor, wall)`](chassis.md#pose-resets) | Corrects one coordinate from a distance sensor facing a wall. |
| [`chassis.resetFromGps()`](chassis.md#pose-resets) | Corrects x and y (and optionally heading) from the GPS. |

## Motions

Each takes an optional [`MotionOptions`](options-results.md#motionoptions) last, such as `{.maxSpeed = 80, .timeoutMs = 1500}`.

| Call | What it does |
|---|---|
| [`chassis.move(distance)`](chassis.md#motions) | Drives straight, holding the heading. Negative = backward. |
| [`chassis.turn(heading)`](chassis.md#motions) | Turns in place to a field heading. |
| [`chassis.turnToPoint(x, y)`](chassis.md#motions) | Turns in place to face a point. |
| [`chassis.pivotTurn(heading, side)`](chassis.md#motions) | Turns about one locked side, or about any robot point with a `LocalOffset`. |
| [`chassis.moveToPoint(x, y)`](chassis.md#motions) | Drives to a field point. Holds its heading on a drive that can strafe. |
| [`chassis.moveToPose(x, y, heading)`](chassis.md#motions) | Drives to a point and ends at a heading. |
| [`chassis.strafe(distance)`](chassis.md#motions) | Slides sideways, positive = right. `Unsupported` on a tank. |
| [`chassis.waitUntil(progress)`](chassis.md#motion-sync) | With `{.async = true}`: waits until the motion has gone this far. |
| [`chassis.waitUntilDone()`](chassis.md#motion-sync) | Waits for the motion to end and returns its result. |
| [`chassis.isMoving()`](chassis.md#motion-sync) | `true` while a motion runs. |
| [`chassis.cancel()`](chassis.md#motion-sync) | Stops the motion; it ends as `Cancelled`. |

## Driver control

| Call | What it does |
|---|---|
| [`chassis.tank(left, right)`](chassis.md#driver-control) | Tank control, one stick per side. |
| [`chassis.arcade(forward, turn)`](chassis.md#driver-control) | Arcade control. |
| [`chassis.holonomic(forward, strafe, turn)`](chassis.md#driver-control) | Forward, sideways and turn on any drive; robot- or field-centric. |
| [`chassis.setDriveMode(mode)`](chassis.md#driver-control) · `driveMode()` | `RobotCentric` (default) or `FieldCentric`. |
| [`chassis.resetFieldHeading()`](chassis.md#driver-control) | Makes the way the robot faces now "forward" for field-centric. |
| [`chassis.fieldCentricIgnored()`](chassis.md#driver-control) | `true` if field-centric was asked for on a drive that can't strafe. |
| [`chassis.setDriverBrake(mode)`](chassis.md#driver-control) · `driverBrake()` | Brake mode when the sticks are centered. |
| [`chassis.driveMillivolts(left, right)`](chassis.md#driver-control) | Raw voltages per side or per wheel, no curves. For tests. |

## Settings

| Call | What it does |
|---|---|
| [`chassis.setGains(gains)`](chassis.md#settings) · `gains()` | Changes the PID tuning at the next motion loop. |
| [`chassis.setAllowDegraded(allow)`](chassis.md#settings) | Lets motions run on degraded odometry. Off by default. |
| [`chassis.config()`](chassis.md#settings) | The config the chassis was built with. |
| [`chassis.drive()`](chassis.md#settings) · `driveKinematics()` | The drive as wheel modules, and each wheel's top speed. |
| [`chassis.odometrySnapshot()`](chassis.md#for-tools-and-the-display) · `motionStatus()`, `motionHistory()`, `telemetry()` | Internal state for the screen, logs and tests. You won't normally call them. |

## Drive presets

| Call | What it does |
|---|---|
| [`cardLib::tank(left, right, wheel, gearing, trackWidth)`](config.md#drive-presets) | A tank drive. |
| [`cardLib::asterisk({...})`](config.md#drive-presets) | Four 45° corners plus two center wheels, from an [`AsteriskConfig`](config.md#asteriskconfig). |
| [`cardLib::customDrive({...})`](config.md#drive-presets) | Any drive, as a list of [`WheelModule`](config.md#wheelmodule)s (X-drive and others). |
| [`cardLib::speedMatch(drive, cartridge)`](config.md#drive-presets) | The drive's top speeds and how much the faster wheels are cut back. |
| [`cardLib::requireSpeedMatchWithin(report, 0.10)`](config.md#drive-presets) | For `static_assert`: stops the build when the wheels disagree too much. |
| [`cardLib::presetKinematics(drive, cartridge)`](config.md#drive-presets) | The full kinematics behind `speedMatch`. Rarely needed. |
| [`cardLib::toDriveModule(module)`](config.md#drive-presets) | Used by the presets. Rarely needed. |

## Config types

| Type | What it holds |
|---|---|
| [`ChassisConfig`](config.md) | The whole robot: `drive`, `sensors`, `calibration`, `gains`, `limits`, `field`. |
| [`DriveConfig`](config.md#driveconfig) | The wheel modules. Build it with a preset. |
| [`SensorConfig`](config.md#sensorconfig) | Tracking wheels, IMU, distance sensors, GPS. Leave out what you don't have. |
| [`Calibration`](config.md#calibration) | Measured corrections from the calibration routines. |
| [`Gains`](config.md#gains) · [`PidGains`](config.md#pidgains) | PID gains and slew rates per motion type. |
| [`Limits`](config.md#limits) | Timeouts, exit windows, brakes, driver curves, warnings. |
| [`FieldWalls`](config.md#fieldwalls) | Where each wall is, for `resetFromWall()`. |
| [`Geometry`](config.md#geometry) | Only for the older `{.left, .right}` tank shorthand. |
| [`MotionOptions`](options-results.md#motionoptions) · [`MotionResult`](options-results.md#motionresult) | Per-motion options, and how a motion ended. |
| [`ResetResult`](options-results.md#resetresult) | How a pose reset went (`Applied` or why not). |

## Hardware

| Call | What it does |
|---|---|
| [`MotorGroup group({{.port = 1}, ...}, "name")`](hardware.md#motorgroup) | Up to 4 motors moving together. Set `reversed` so positive drives forward. |
| [`group.moveMillivolts(mv)`](hardware.md#motorgroup) · `stop(mode)` | Drives a mechanism directly. |
| [`group.readDegrees()`](hardware.md#motorgroup) · `velocityRpm(i)`, `currentAmps(i)` | Reads the group, empty if no motor answers. |
| [`Imu imu(port)`](hardware.md#sensors) | The inertial sensor. Pass it in an `ImuConfig`. |
| [`RotationSensor(port, reversed)`](hardware.md#sensors) · `AdiEncoder(top, bottom, reversed)` | Tracking wheel sensors. |
| [`DistanceSensor(port)`](hardware.md#sensors) · `GpsSensor(port)` | For wall and GPS resets. |
| [`AdiDigitalOut(port)`](hardware.md#sensors) · `set(on)`, `toggle()` | Pneumatics and other ADI outputs. |
| `Omniwheel::NEW_325` · [`Gearing{36, 48}`](hardware.md#wheels-and-gearing) | Measured wheel sizes and external gearing. |
| [`smartPortsValid(ports)`](hardware.md#port-checks) · `adiPortsValid(ports)` | For `static_assert`: ports in range, no duplicates. |

## Screen and tools

| Call | What it does |
|---|---|
| [`Display(chassis, tests, blackBox)`](systems.md#display) · `start()` | The brain-screen pages and the auton selector. |
| [`display.setAutons(list)`](systems.md#display) · `runSelectedAuton()` | Lists your autons; runs the chosen one from `autonomous()`. |
| [`TestModes(chassis)`](systems.md#testmodes) · `runPending()` | Port check, sign check and tape box, started from the Tests page. |
| [`CalibrationRoutines(chassis)`](systems.md#calibrationroutines) · `printResults()` | Measures wheel sizes, offsets and IMU scale; prints code to paste. |
| [`CalibrationFile::load(path, calibration)`](systems.md#calibrationfile) | Loads calibration values from the SD card. |
| [`TerminalLog(chassis.telemetry())`](systems.md#terminallog-and-blackbox) · `BlackBox(chassis)` | Telemetry as CSV in the terminal; the last 64 motions on SD. |
