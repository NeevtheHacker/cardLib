# cardLib

cardLib is a PROS 4 library for VEX V5 robots. It tracks where your robot is on the field, drives it there with tuned PID motions, handles driver control, and tells you clearly when a sensor or port is wrong.

It ships two ways, like LemLib:

- **A PROS template** (`cardLib@0.1.0.zip`) you apply to your own project.
- **A starter project** that already has cardLib applied, with folders for your robot, mechanisms, driver control and autons.

```cpp
#include "cardLib/api.hpp"

void autonomous() {
  chassis.setPose(0, 0, 0);                         // start at the origin, facing +y
  chassis.moveToPoint(24, 24, {.maxSpeed = 90});    // drive to (24, 24)
  chassis.turn(90);                                 // face the +x direction
  chassis.moveToPose(48, 24, 90, {.timeoutMs = 2000});
}
```

## What you get

| Feature | Details |
|---|---|
| Odometry | Arc odometry from tracking wheels, IMU and drive encoders, with automatic fallbacks when a sensor fails |
| Motions | `move`, `turn`, `turnToPoint`, `pivotTurn`, `moveToPoint`, `moveToPose` |
| Chaining | `async`, `waitUntil`, `waitUntilDone`, `minSpeed` + `earlyExit` so motions flow into each other |
| Every motion reports how it ended | `Settled`, `Stalled`, `TimedOut`, `Cancelled`, `SensorFault`, `NotCalibrated` |
| Driver control | Tank and arcade with deadzone and expo curves |
| Calibration | Routines that measure your real wheel sizes, offsets and IMU scale and print copy-paste code |
| Diagnostics | Port and device check at startup, test modes, brain-screen pages, terminal telemetry, a black box on SD |
| Safety | Optional sensors are optional; required ones block precision motions instead of guessing |

!!! info "New in 0.2"
    Holonomic drives (asterisk, X-drive and custom layouts) with one drivetrain model, strafing, field-centric driver control, distance-sensor wall resets, a GPS reset and a Sensors page. See [Holonomic drives](tutorials/10-holonomic.md).

## Where to start

1. [Install cardLib](getting-started/installation.md) into a new or existing PROS project.
2. Follow [Your first project](getting-started/first-project.md) to describe your robot.
3. Work through the tutorials in order. Each one ends with something you can test on the robot.

## Conventions in one line

Inches, degrees and milliseconds everywhere. +x is right, +y is forward, heading 0 points along +y and increases clockwise (the same frame as LemLib). Speeds are −127 to 127. Details in [Conventions](tutorials/1-conventions.md).
