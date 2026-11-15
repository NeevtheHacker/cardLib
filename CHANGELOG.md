# Changelog

cardLib uses `MAJOR.MINOR.PATCH`. A change that breaks your code bumps MAJOR, a new feature bumps
MINOR, a fix bumps PATCH. The Makefile `VERSION`, the release tag and this file always match.
This file mirrors `docs/project/changelog.md`.

## Unreleased (planned as 0.2.0)

### Breaking

- `Geometry` has no default wheel diameter or track width any more (they were 3.25 in and
  12 in), and `ChassisConfig::geometry` is now optional: set it only for the `{.left, .right}`
  tank shorthand, as `config.geometry = cardLib::Geometry{.wheelDiameterInches = ...,
  .trackWidthInches = ...}`. Leaving either value out is a compile error. Configs built with
  `tank()`, `asterisk()` or `customDrive()` don't set it. The shorthand without `geometry`
  starts with a "no drive" startup fault.
- `TrackingWheelConfig::diameterInches` has no default any more (it was 2.75 in). Use an
  `Omniwheel::` constant or your push-test value.

### Added

- One drivetrain model for every drive: wheel modules with a diameter, gearing, position and roll
  angle. `cardLib::tank()`, `cardLib::asterisk()` and `cardLib::customDrive()` presets. The old
  `{.left, .right}` plus `Geometry` tank setup still works (see Breaking).
- `Omniwheel` measured wheel-size constants, `Gearing` as `{motorTeeth, wheelTeeth}` or a ratio,
  and 5.5 W motors at 200 rpm.
- Drive kinematics computed at compile time, with a speed-match report and
  `requireSpeedMatchWithin()` for `static_assert`.
- Desaturation with rotation priority on every drive; per-wheel drive commands.
- `strafe()`, holonomic `moveToPoint()` / `moveToPose()` (or `{.tankStyle = true}`), and
  `pivotTurn()` about any robot-local point. `MotionResult::Unsupported` on a drive that can't
  make the motion.
- `holonomic()` driver control, robot-centric by default; `setDriveMode()`,
  `resetFieldHeading()` and a per-call `{.fieldCentric = true}`. Field-centric curves the stick's
  length before turning it, so it feels the same at any heading.
- `RobotState` gains `sidewaysVelocityInchesPerSecond`; drive-encoder odometry for holonomic
  drives through least-squares forward kinematics.
- `RobotState::driveSlipSuspect`: on a drive with more wheels than it needs, a warning when the
  wheels disagree with each other (the forward-kinematics residual), shown as "wheel slip?".
- The port and direction check spins each wheel module alone, and the Tuner page shows its
  voltages.
- Calibration on holonomic drives: the push run's drive scale comes from forward kinematics;
  the effective track width is skipped there.
- Startup reports a wheel module with no motor group, and a config with no drive, as faults.
- `GpsSensor`, `resetFromWall()` and `resetFromGps()`, and a Sensors page showing every sensor as
  OK, Suspect, Faulted, Wrong or Missing.
- `LICENSE`, `README.md`, this changelog, `.gitignore` and fuller `.gitattributes`.
- The documentation site config (`mkdocs.yml`, GitHub Pages workflow), an "At a glance" page
  listing every function a team calls, a "Checking a new robot" page, and `tools/check_docs.py`,
  which fails when the reference misses a public `Chassis` function or drive preset.
- Laptop tests now run the chassis code (driver control, resets, sensor status, the port check)
  against fake PROS tasks and devices.

## 0.1.0

First version.

- Arc odometry with tracking wheels, IMU and drive encoders, plausibility checks and fallbacks.
- Motions: `move`, `turn`, `turnToPoint`, `pivotTurn`, `moveToPoint`, `moveToPose`, with async,
  `waitUntil`, `minSpeed` and `earlyExit`.
- Tank and arcade driver control with curves.
- Startup port and device validation.
- Calibration routines and optional SD calibration file.
- Display pages, test modes, live PID tuner, terminal telemetry, black box.
- Starter project with one file per auton.
