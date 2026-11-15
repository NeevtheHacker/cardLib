# 10. Holonomic drives

!!! note "New in 0.2"
    Everything on this page arrives in cardLib 0.2. Tank robots don't need any of it: a tank is just the drive whose wheels all point forward, and the [Drivetrain setup](2-drivetrain.md) keeps working unchanged.

cardLib uses one drivetrain model for every drive. The same `Chassis`, motions and driver control run a tank, an asterisk, an X-drive or any custom layout. You describe the wheels; cardLib works out the rest.

## Wheel modules

Each driven wheel is a **module**: its motor group, wheel size, gearing, position and the direction it pushes.

| Field | Meaning |
|---|---|
| `motors` | The motor group driving this wheel |
| `wheelDiameterInches` | Real rolling diameter, from `Omniwheel::` or measured. **Required** |
| `gearing` | `{motorTeeth, wheelTeeth}` such as `{36, 48}`, or a plain ratio such as `0.75`. **Required** |
| `xInches`, `yInches` | Wheel contact point from the robot center (+x right, +y forward) |
| `rollAngleDegrees` | Which way a positive command pushes the robot, clockwise from forward. **Required** |

The required fields have no default: leaving one out is a compile error, never a guess. The cartridge comes from the motor group (`Blue600`, `Green200`, `Red100`; 5.5 W motors always run at 200 rpm).

## Wheel sizes

| Constant | Inches | | Constant | Inches |
|---|---|---|---|---|
| `NEW_2` | 2.125 | | `NEW_4` | 4.00 |
| `NEW_275`, `OLD_275` | 2.75 | | `OLD_4` | 4.18 |
| `NEW_275_HALF` | 2.744 | | `NEW_4_HALF` | 3.995 |
| `OLD_275_HALF` | 2.74 | | `OLD_4_HALF` | 4.175 |
| `NEW_325`, `OLD_325` | 3.25 | | | |
| `NEW_325_HALF`, `OLD_325_HALF` | 3.246 | | | |

These are LemLib's approximate measured diameters. After calibration, replace them with your own push-test value: a plain number such as `3.231` works anywhere a constant does.

## The corner standard

- A positive command on **any** wheel pushes the robot forward. Set each motor's `reversed` flag so this is true.
- Module order: front-left, front-right, back-left, back-right, then center-left, center-right.
- Front-left and back-right roll at +45° (forward-right); front-right and back-left at −45° (forward-left).
- Strafe right = FL +, FR −, BL −, BR +. Clockwise turn = every left wheel +, every right wheel −.

The port and direction check on the Tests page spins each module alone, wheels off the ground, and prints which way it should push the robot. Run it before anything else.

## Presets

Put the drive in your robot's setup file (in the starter, `robot/chassis_setup.cpp`). Every number below is an **example** for one robot; measure your own.

```cpp
// Tank: one module per side.
config.drive = cardLib::tank(leftDrive, rightDrive, cardLib::Omniwheel::NEW_325, {36, 48}, 12.25);

// Asterisk: four 45° corners plus two forward center wheels.
config.drive = cardLib::asterisk({
    .corners = {&frontLeft, &frontRight, &backLeft, &backRight},
    .cornerWheel = cardLib::Omniwheel::NEW_275,
    .cornerGearing = {36, 48},
    .cornerHalfWidthInches = 5.5,   // example
    .cornerHalfLengthInches = 5.0,  // example
    .centers = {&centerLeft, &centerRight},
    .centerWheel = cardLib::Omniwheel::NEW_275,
    .centerGearing = {1, 1},
    .centerHalfWidthInches = 6.0,   // example
});

// X-drive, or anything else: list the modules yourself, in the standard order.
config.drive = cardLib::customDrive({
    {&frontLeft,  cardLib::Omniwheel::NEW_325, {1, 1}, -6.0,  6.0,  45.0},  // example positions
    {&frontRight, cardLib::Omniwheel::NEW_325, {1, 1},  6.0,  6.0, -45.0},
    {&backLeft,   cardLib::Omniwheel::NEW_325, {1, 1}, -6.0, -6.0, -45.0},
    {&backRight,  cardLib::Omniwheel::NEW_325, {1, 1},  6.0, -6.0,  45.0},
});
```

The braced fields of `customDrive` are `{motors, wheelDiameterInches, gearing, xInches, yInches, rollAngleDegrees}`.

## Speed matching, checked when the code compiles

A 45° wheel only rolls at cos 45° ≈ 0.707 of the robot's forward speed, so corner wheels can often push faster than straight wheels at the same rpm. cardLib works out each wheel's top speed and the drive's top forward, strafe and turn speeds, and trims the faster wheels so none drag the others:

```text
maxForward = smallest over all wheels of  (π · D · wheelRpm / 60) / |cos(rollAngle)|
```

**Example** (one asterisk): 2.75 in omnis, corners at 450 rpm (blue, 36:48), centers at 600 rpm (blue, direct). The corners could give 91.6 in/s forward and the centers 86.4 in/s, so the robot tops out at 86.4 in/s with the corners at 94.3 %. Strafing uses only the corners and reaches 91.6 in/s.

To stop the build when a gear ratio or wheel size is mistyped, declare the drive `constexpr` and add a `static_assert`:

```cpp
constexpr cardLib::DriveConfig DRIVE = cardLib::asterisk({ /* as above */ });
static_assert(cardLib::requireSpeedMatchWithin(
                  cardLib::speedMatch(DRIVE, cardLib::Cartridge::Blue600), 0.10),
              "corner and center wheels disagree by more than 10%");
```

`speedMatch(drive, cartridge)` returns a `SpeedMatchReport` with `maxForwardInchesPerSecond`, `maxStrafeInchesPerSecond`, `maxSpinRadiansPerSecond`, each wheel's `forwardDuty` and the overall `mismatch`. The cartridge is passed in because a motor group can't be read at compile time; for 5.5 W motors pass `cardLib::MotorType::Exp5_5W` as a third argument.

## Holonomic motions

On a drive that can move sideways, `moveToPoint` and `moveToPose` run holonomically by themselves.

| Motion | On a holonomic drive | On a tank |
|---|---|---|
| `strafe(distanceInches)` | Slides sideways (positive = right), holding the heading | `Unsupported` |
| `moveToPoint(x, y)` | Translates straight there, holding the start heading (or `.headingDegrees`) | Turns and drives, as before |
| `moveToPose(x, y, θ)` | Translates and turns at the same time; settles when both have | Drive, then turn, as before |
| `pivotTurn(θ, LocalOffset{right, forward})` | Turns about any robot-local point | Only points on the axle line (`forward = 0`); others are `Unsupported` |
| any of the above with `{.tankStyle = true}` | Uses the tank version | Same |

```cpp
chassis.strafe(24);                                       // 24 in to the right
chassis.moveToPoint(24, 48, {.headingDegrees = 90});      // end facing 90°, path straight
chassis.moveToPose(0, 0, 180);                            // move and turn together
chassis.pivotTurn(90, cardLib::LocalOffset{6.0, 6.0});    // about the front-right corner
chassis.moveToPoint(24, 48, {.tankStyle = true});         // the tank way, on purpose
```

`strafe` uses `gains.strafe` and `gains.strafeSlew`; holonomic `moveToPoint` uses the linear gains with heading hold, and holonomic `moveToPose` uses the turn gains for its heading.

A motion that returns `Unsupported` never starts. It is still recorded in the black box, so a wrong-drive auton shows up there.

## Driver control

```cpp
chassis.holonomic(forward, strafe, turn);                          // uses the current drive mode
chassis.holonomic(forward, strafe, turn, {.fieldCentric = true});  // this call only
chassis.tank(left, right);  chassis.arcade(forward, turn);         // still work on any drive
```

**Robot-centric** (the default on every drive): stick forward moves the robot the way it faces. **Field-centric**: stick forward always moves the robot away from the driver, whichever way it faces. Field-centric falls back to robot-centric by itself if the IMU faults.

A button for each is a common setup. Example, in your driver loop (the starter is a tank, so it doesn't include these):

```cpp
void opcontrol() {
  while (true) {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {   // example button
      const bool field = chassis.driveMode() == cardLib::DriveMode::FieldCentric;
      chassis.setDriveMode(field ? cardLib::DriveMode::RobotCentric
                                 : cardLib::DriveMode::FieldCentric);
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {   // example button
      chassis.resetFieldHeading();  // "forward" is now the way the robot faces
    }
    chassis.holonomic(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                      master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),
                      master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    pros::delay(10);
  }
}
```

On a tank, field-centric is ignored: `chassis.fieldCentricIgnored()` turns `true` and the Diagnostics page says so.

## Odometry on a holonomic robot

A holonomic robot really moves sideways, so it needs a **horizontal tracking wheel** for full precision. Without one, sideways motion is estimated from the drive wheels through forward kinematics and odometry is always `Degraded`. Precision motions refuse to run on degraded odometry unless you allow it:

```cpp
chassis.setAllowDegraded(true);                         // every motion
chassis.moveToPoint(24, 24, {.allowDegraded = true});   // this motion only
```

With more drive wheels than it needs (any X-drive or asterisk), cardLib also checks that the wheels agree with each other. When one slips or lifts for a moment, `getState().driveSlipSuspect` turns on and the Pose page shows `wheel slip?`. It is a warning only; the pose isn't changed.

`RobotState` has `sidewaysVelocityInchesPerSecond` (to the right) as well as the forward and turn velocities.

## Calibration on a holonomic robot

The [calibration routines](4-calibration.md) work on any drive. Two differ:

- **Drive scale** (the push run) measures how far the robot went through forward kinematics over every module, instead of the left/right average.
- **Effective track width** isn't measured; a holonomic drive's encoder heading comes from forward kinematics and doesn't use it. `printResults()` says so.

## Wall and GPS resets

Distance-sensor wall resets and GPS resets work on every drive; see the [Chassis reference](../reference/chassis.md#pose-resets).
