# 2. Drivetrain setup

This page sets up a **differential (tank) drivetrain**: one motor group per side. Asterisk, X-drive and custom layouts use the same model; see [Holonomic drives](10-holonomic.md).

## Motor groups

```cpp
cardLib::MotorGroup leftDrive({
    {.port = 1, .reversed = true,  .cartridge = cardLib::Cartridge::Blue600},
    {.port = 2, .reversed = true,  .cartridge = cardLib::Cartridge::Blue600},
    {.port = 3, .reversed = false, .cartridge = cardLib::Cartridge::Blue600},  // stacked motor
}, "left drive");
```

- **`reversed`**: set it so a positive command drives the robot **forward**. Never guess: the Port / direction test checks it.
- **`cartridge`**: must match the cartridge physically inside the motor (`Red100`, `Green200`, `Blue600`). cardLib never infers it.
- If one motor in a group unplugs mid-match, the group keeps reporting position from the motors that are still answering.

## The drive

```cpp
// Example values; measure your own.
config.drive = cardLib::tank(leftDrive, rightDrive, cardLib::Omniwheel::NEW_325, {36, 48}, 12.25);
```

| Argument | What to enter |
|---|---|
| wheel diameter | An `Omniwheel::` constant (`NEW_325`, `OLD_4`, ...) to start; calibration then gives you the measured one. |
| gearing | External gears only, `{motorTeeth, wheelTeeth}`: 36 teeth on the motor driving 48 on the wheel is `{36, 48}`. Direct drive is `{1, 1}`. |
| track width | Distance between the left and right wheel contact lines, measured center to center, in inches. |

None of these has a default: leave one out and the code doesn't compile. The cartridge is already in the motor config, so the gearing only counts gears *outside* the motor. A blue motor through 36:48 runs the wheel at 600 × 0.75 = 450 rpm.

!!! tip "Real wheel sizes"
    VEX wheels are not exactly their advertised size. Old-style 4 in omnis measure about 4.18 in, and half-cut wheels are a few thousandths smaller. The `Omniwheel` constants are LemLib's measured values, and the library's calibration measures your own (see [Calibration](4-calibration.md)).

??? note "The older left/right setup"
    `config.drive.left`/`right` plus `config.geometry = cardLib::Geometry{.wheelDiameterInches = ..., .trackWidthInches = ..., .externalRatio = ...}` still works. Both numbers are required; without `geometry` the robot starts with a "no drive" fault.

## Driver brake and motion brake

```cpp
config.limits.motionBrake = cardLib::BrakeMode::Brake;  // after a motion settles
config.limits.driverBrake = cardLib::BrakeMode::Coast;  // when the sticks are centered
```

Change the driver brake during a match with `chassis.setDriverBrake(cardLib::BrakeMode::Hold)`.

## Checking it

Wheels off the ground, run the Port / direction test. Every motor should spin the direction the screen shows. Then drive with arcade and check that forward is forward and right turns right.
