# cardLib

cardLib is a PROS 4 library for VEX V5 robots. It gives you odometry, PID motions, driver
control and on-robot diagnostics, for tank drives and holonomic drives (asterisk, X-drive or any
wheel layout) through one drivetrain model.

- **Odometry**: arc odometry from tracking wheels, the IMU and drive encoders, with plausibility
  checks and fallbacks when a sensor drops out. Wall resets from distance sensors and a GPS reset.
- **Motions**: `move`, `turn`, `turnToPoint`, `pivotTurn`, `moveToPoint`, `moveToPose`, plus
  `strafe` and holonomic `moveToPoint` / `moveToPose` on drives that can strafe. Every motion has
  a timeout and returns a result.
- **Driver control**: `tank`, `arcade` and `holonomic` (robot-centric or field-centric), with
  deadzones and expo curves.
- **Diagnostics**: startup port checks, a port and direction check per wheel, sensor sign
  checks, calibration routines, a live PID tuner, a sensor status page and a black box.

The frame is LemLib's: +x right, +y forward, heading in degrees, 0 = +y, clockwise positive. The
public API uses inches, degrees, milliseconds and speeds from 0 to 127.

## Install

Download `cardLib@0.1.0.zip` from the Releases page, then in your PROS project:

```sh
cd my-robot
pros c fetch cardLib@0.1.0.zip
pros c apply cardLib
```

Or start from `cardLib-starter@0.1.0.zip`, a complete project with cardLib applied, one file per
auton and an auton selector.

## A short example

Every port and number below is an example. Measure your own robot.

```cpp
#include "main.h"
#include "cardLib/api.hpp"

// Example robot: a tank drive on 3.25 in omnis, 36:48 gearing, 12.25 in track width.
cardLib::MotorGroup leftDrive({{.port = 1, .reversed = true}, {.port = 2, .reversed = true}},
                              "left drive");
cardLib::MotorGroup rightDrive({{.port = 3}, {.port = 4}}, "right drive");
cardLib::Imu imu(10);
const cardLib::ImuConfig imuConfig{.sensor = &imu};

cardLib::ChassisConfig makeConfig() {
  cardLib::ChassisConfig config;
  config.drive =
      cardLib::tank(leftDrive, rightDrive, cardLib::Omniwheel::NEW_325, {36, 48}, 12.25);
  config.sensors.imu = &imuConfig;
  return config;
}

cardLib::Chassis chassis(makeConfig());

void initialize() { chassis.initialize(); }

void autonomous() {
  chassis.setPose(0, 0, 0);
  chassis.move(24);
  chassis.turn(90);
  if (chassis.moveToPose(24, 24, 90, {.maxSpeed = 80, .timeoutMs = 2000}) ==
      cardLib::MotionResult::Stalled) {
    chassis.move(-6);
  }
}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  while (true) {
    chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                   master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    pros::delay(10);
  }
}
```

## Documentation

The guides and API reference are in [`docs/`](docs/index.md), built with MkDocs Material and
published to GitHub Pages. Start with [Installation](docs/getting-started/installation.md) and
[Your first project](docs/getting-started/first-project.md). Holonomic drives are covered in
[tutorial 10](docs/tutorials/10-holonomic.md).

## Repository layout

| Folder | What it is |
| --- | --- |
| `cardLib/` | The library, a PROS project. Run every `pros` command from inside it. |
| `cardLib/tests/` | Laptop tests (doctest): `cmake -S tests -B build/tests && cmake --build build/tests && ctest --test-dir build/tests` |
| `starter/` | The starter project, with cardLib applied. Refresh it with `tools/refresh_starter.sh`. |
| `docs/` | The documentation site (`mkdocs.yml` at the root). |
| `tools/` | Maintainer scripts, such as `refresh_starter.sh`. |

## License and credits

MIT, see [LICENSE](LICENSE). cardLib builds on ideas from LemLib, EZ-Template, OkapiLib,
5225A, WPILib and others; see [CREDITS.md](CREDITS.md). Changes are listed in
[CHANGELOG.md](CHANGELOG.md).
