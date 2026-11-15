# 4. Calibration

Calibration measures the numbers you can't measure well with a ruler: the IMU's scale, your wheels' real rolling diameter, and the exact tracking-wheel offsets. The routines print C++ you paste into `robot/calibration_values.hpp`.

!!! note "Physical facts vs measured corrections"
    The drive config (`tank()`, `asterisk()` or `customDrive()`) and the sensor configs hold what the robot is built from (wheel sizes, track width, gears, positions). `Calibration` holds measured corrections. Rebuild the robot → change the drive and sensor configs. Re-run calibration → change calibration. Keeping them apart means you always know where a number came from.

## Order

Run these in order: later steps use earlier results.

| Step | Measures | How |
|---|---|---|
| 1. IMU scale | `imuScale` | Spin the robot a known number of full turns against a wall or line |
| 2. Push test | Vertical wheel diameter, `driveScale` | Push the robot straight along a taped distance (e.g. 72 in) |
| 3. Sideways push | Horizontal wheel diameter | Push sideways along a taped distance |
| 4. Spin test | Tracking-wheel offsets, effective track width | Spin in place several times |
| 5. Distance sensors | `gain`, `offsetInches` | Readings at known distances from a wall |

## Running them

```cpp
cardLib::CalibrationRoutines calibrationRoutines(chassis);

void calibrateRobot() {
  calibrationRoutines.beginRun();
  calibrationRoutines.spin(10, cardLib::TurnDirection::Clockwise);   // 10 turns
  calibrationRoutines.recordImuRun(10);

  calibrationRoutines.beginRun();
  // push the robot 72 in by hand along tape, then:
  calibrationRoutines.recordPushRun(72);

  calibrationRoutines.beginRun();
  calibrationRoutines.spin(5, cardLib::TurnDirection::CounterClockwise);
  calibrationRoutines.recordSpinRun();

  calibrationRoutines.printResults();   // open `pros terminal` to see it
}
```

Do each run at least twice; `printResults()` marks a value whose runs disagree, so you know to repeat it.

## Pasting the results

```cpp
inline cardLib::Calibration robotCalibration() {
  cardLib::Calibration calibration;
  calibration.imuScale = 1.0094;                    // 10 turns CW + CCW, 11/15
  calibration.driveScale = 0.9871;                  // 72 in push x3, 11/15
  calibration.verticalWheelDiameterInches[0] = 2.738;
  calibration.verticalWheelXInches[0] = -0.42;
  calibration.source = "code 11/15";
  return calibration;
}
```

(The numbers above are examples.) Any value left empty falls back to the drive or sensor config. On a holonomic drive, see [Calibration on a holonomic robot](10-holonomic.md#calibration-on-a-holonomic-robot).

## Optional SD card file

Values in code are the default. A file at `/usd/cardLib/robot.cal` overrides only the keys it contains, which is handy at competitions. Load it before `initialize()`:

```cpp
cardLib::Calibration calibration = chassis.config().calibration;
if (cardLib::CalibrationFile::load("/usd/cardLib/robot.cal", calibration) ==
    cardLib::CalibrationFile::LoadResult::Loaded) {
  chassis.setCalibration(calibration);
}
```

## Checking it: the tape box

Drive a square (or any path that comes back to the start) and compare where odometry thinks it ended with where it really is. The **Tape box** test on the Tests page does this and prints the error. Do it at half speed and full speed. Under about an inch after a full lap is good.
