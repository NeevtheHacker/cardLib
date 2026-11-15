# Checking a new robot

Run these checks on any new robot, and again after rewiring or rebuilding the drive. Do them in order: each one trusts the ones before it. They take about half an hour, and they catch the mistakes that otherwise look like bad tuning: a motor running backwards, a sensor with the wrong sign, a wrong wheel size.

Write down for every run: the date, battery %, the result the call returned and what you measured with a tape.

## Before you start

- The code builds with no warnings (`pros make`).
- Brain screen, **Diagnostics** page: `Startup: Ready`, and no port lines, "no drive" or "modules with no motors".
- **Sensors** page: every configured sensor shows OK. A distance sensor with nothing in range shows Suspect; that's fine.

If something is listed, or a sensor shows Wrong or Missing, fix the port or the config first. See [Sensors](../tutorials/3-sensors.md#the-sensors-page).

## 1. Direction check

Wheels **off the ground**, robot on a stand.

1. Tests page → Port / direction → run it (or call `testModes.runPortCheck()`).
2. Each wheel module spins alone for 1 s at 10 %. The terminal prints one line per module, saying which way it should push the robot, then one line per motor.

Pass when, for **every** module:

- The wheel that spins is the one the line describes (front-left first, then front-right, back-left, back-right, then the centers).
- Held against the floor, the wheel would push the robot the way the line says (+45° = forward-right, −45° = forward-left, 0° = forward).
- Every motor line says `direction ok` and `speed ok`.

If a module spins backwards, flip its motors' `reversed` flags; never change the roll angle. If `SPEED?` shows, the cartridge in the config doesn't match the motor.

## 2. Sign check

Robot on the field. Tests page → Sign check starts it; push the robot by hand, then run it again to stop. It shows how far each sensor moved in between. Do it three times:

- Pushed straight forward: every vertical tracking wheel and the drive wheels read positive.
- Turned clockwise in place: the IMU rotation reads positive.
- Pushed right (only with a horizontal tracking wheel): the horizontal wheel reads positive.

A wrong sign here is a `reversed` flag on a tracking wheel or the IMU orientation in its config. Fix it in the config; never flip a sign in your own code to compensate.

## 3. Calibration

Measure the real wheel sizes, tracking-wheel offsets and IMU scale with the calibration routines, then paste the printed values into your code. The order and the steps are in [Calibration](../tutorials/4-calibration.md). Until this is done, odometry uses the nominal `Omniwheel::` sizes and will be off by a few percent.

Then check the result with the **tape box** test: drive a closed loop and compare where odometry thinks it ended with where the robot really is. Under about an inch after a full lap is good. See [Checking it: the tape box](../tutorials/4-calibration.md#checking-it-the-tape-box).

## 4. Drives that strafe

Skip this on a tank (`chassis.strafe(24)` returns `Unsupported` there and doesn't move).

1. `chassis.setPose(0, 0, 0);` at a taped start line, then `chassis.strafe(24)` several times right and `chassis.strafe(-24)` several times left, re-placing the robot each run.
2. Each run should return `Settled`, end within about an inch of 24 in sideways, drift under an inch forward and keep its heading within about 2°.
3. The Pose page should never show `wheel slip?` on a clean run.

With field-centric driving (see [Holonomic drives](../tutorials/10-holonomic.md)): turn the robot 90° clockwise, switch to field-centric and push the stick forward. The robot should move away from the driver while still facing right.

## 5. Wall resets

Only if you use `resetFromWall()`. Distance sensor calibrated (calibration step 5), its position in `DistanceSensorConfig` measured, `config.field` matching your field.

1. Put the robot square to a wall at a taped distance, and `setPose` a value 2 in wrong in that axis.
2. Robot still: `chassis.resetFromWall(index, cardLib::Wall::Back)` (or the wall it faces).

It should return `Applied`, and the Pose page should then match the tape within about half an inch. Only that one coordinate changes. Rotated 20° off square it returns `NotSquare`; called while driving, `NotStopped`. Either way the pose is left alone.

## 6. GPS frame check

**Required before any auton uses `resetFromGps()`.** cardLib does the GPS mount offset and field rotation itself, and assumes the GPS reports +x right, +y forward and heading clockwise. This check proves that on your field.

1. Mark two taped points at least 24 in apart in both x and y, about 20 in from any wall: for example (−24, −24) and (24, 24) in your frame.
2. At the first point facing 0°: wait 2 s still, then `chassis.resetFromGps({.resetHeading = true})`. Record the result and the pose.
3. At the second point facing 90°: the same.

Pass when:

- Both return `Applied`.
- Each pose is within about 1 in of the taped point and 3° of the heading.
- x and y aren't swapped (moving the robot right changes x, not y).
- Neither axis is mirrored (the first point's signs are right).
- The heading isn't reversed (facing 90° reads 90, not 270).

If x and y are swapped, an axis is mirrored or the heading is reversed, **stop** and report it with what you saw. Don't try to fix it with `fieldRotationDegrees` or the offsets.

Every limit above ("about an inch", "2°") is a starting point. Tighten it once you know what your robot does.
