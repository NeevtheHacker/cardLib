# Changelog

cardLib uses `MAJOR.MINOR.PATCH`. A change that breaks your code bumps MAJOR, a new feature bumps
MINOR, a fix bumps PATCH. The Makefile `VERSION`, the release tag and this file always match.
This file mirrors `docs/project/changelog.md`.

## Unreleased (planned as 0.2.0)

- One drivetrain model for every drive: wheel modules with a diameter, gearing, position and roll
  angle. `cardLib::tank()`, `cardLib::asterisk()` and `cardLib::customDrive()` presets. The old
  `{.left, .right}` plus `Geometry` tank setup still works.
- `Omniwheel` measured wheel-size constants, `Gearing` as `{motorTeeth, wheelTeeth}` or a ratio,
  and 5.5 W motors at 200 rpm.
- Drive kinematics computed at compile time, with a speed-match report and
  `requireSpeedMatchWithin()` for `static_assert`.
- Desaturation with rotation priority on every drive; per-wheel drive commands.
- `strafe()`, holonomic `moveToPoint()` / `moveToPose()` (or `{.tankStyle = true}`), and
  `pivotTurn()` about any robot-local point. `MotionResult::Unsupported` on a drive that can't
  make the motion.
- `holonomic()` driver control, robot-centric by default; `setDriveMode()`,
  `resetFieldHeading()` and a per-call `{.fieldCentric = true}`.
- `RobotState` gains `sidewaysVelocityInchesPerSecond`; drive-encoder odometry for holonomic
  drives through least-squares forward kinematics.
- The port and direction check spins each wheel module alone.
- `GpsSensor`, `resetFromWall()` and `resetFromGps()`, and a Sensors page showing every sensor as
  OK, Suspect, Faulted or Missing.
- `LICENSE`, `README.md`, this changelog, `.gitignore` and fuller `.gitattributes`.

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
