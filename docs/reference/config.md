# Configuration

`struct cardLib::ChassisConfig` groups its parts by why they change.

```cpp
struct ChassisConfig {
  DriveConfig drive;                  // the wheels: build it with tank(), asterisk() or customDrive()
  SensorConfig sensors;               // which sensors
  std::optional<Geometry> geometry;   // only for the {.left, .right} tank shorthand
  Calibration calibration;            // measured corrections: change when calibration is re-run
  Gains gains;                        // tuning
  Limits limits;                      // behaviour and preferences
  FieldWalls field;                   // where resetFromWall() finds each wall
};
```

cardLib never contains one team's numbers. Values a drive can't work without (wheel diameter, gearing, roll angle, track width, tracking wheel diameter) have **no default**: leaving one out is a compile error. Every other default is neutral (ratio 1, offsets 0, scales 1, no optional sensors).

## DriveConfig

Build it with a preset; see [Holonomic drives](../tutorials/10-holonomic.md#presets).

```cpp
config.drive = cardLib::tank(leftDrive, rightDrive, cardLib::Omniwheel::NEW_325, {36, 48}, 12.25);  // example
```

| Field | Default | Meaning |
|---|---|---|
| `std::array<DriveModule, 8> modules`, `moduleCount` | none | The wheel modules, in the standard order |
| `MotorGroup* left`, `right` | `nullptr` | The tank shorthand (needs `geometry`); also set by `tank()` |

A `DriveModule` is `{motors, wheelDiameterInches, gearRatio, xInches, yInches, rollAngleDegrees}`. Presets build it from a `WheelModule`, whose diameter, gearing and roll angle are required.

## SensorConfig

| Field | Default |
|---|---|
| `const TrackingWheelConfig* verticalWheel1`, `verticalWheel2` | `nullptr` |
| `const TrackingWheelConfig* horizontalWheel` | `nullptr` |
| `const ImuConfig* imu` | `nullptr` |
| `std::array<const DistanceSensorConfig*, 4> distanceSensors` | all `nullptr` |
| `const GpsConfig* gps` | `nullptr` |

Leave out anything the robot doesn't have.

## Geometry

Only for the older `{.left, .right}` tank shorthand; a drive built with a preset carries its own numbers and leaves `geometry` empty. Without it, the shorthand starts with a "no drive" fault.

```cpp
config.drive.left = &leftDrive;
config.drive.right = &rightDrive;
config.geometry = cardLib::Geometry{.wheelDiameterInches = 3.25, .trackWidthInches = 12.0};  // example
```

| Field | Default | Meaning |
|---|---|---|
| `wheelDiameterInches` | **required** | Drive wheel diameter |
| `trackWidthInches` | **required** | Left-to-right wheel distance |
| `externalRatio` | `1.0` | Wheel turns per motor output turn (external gears only) |

## Calibration

| Field | Default | Meaning |
|---|---|---|
| `imuScale` | `1.0` | IMU heading correction |
| `driveScale` | `1.0` | Drive-encoder distance correction |
| `effectiveTrackWidthInches` | empty | Measured track width for encoder heading |
| `verticalWheelXInches[i]`, `verticalWheelDiameterInches[i]` | empty | Measured per vertical wheel |
| `horizontalWheelYInches`, `horizontalWheelDiameterInches` | empty | Measured for the horizontal wheel |
| `source` | `"code"` | Shown on screen, e.g. `"SD 11/15"` |

Empty means "not measured; use the geometry or sensor config".

## Gains

| Field | Default | Used by |
|---|---|---|
| `PidGains linear` | `kP 10` | `move`, `moveToPoint` distance |
| `PidGains heading` | `kP 2` | Heading hold during moves |
| `PidGains turn` | `kP 2` | Turns, `moveToPose` final turn |
| `PidGains pivot` | `kP 3` | `pivotTurn` |
| `SlewRates linearSlew` | 500 / 1500 per s | Linear output rate limit |
| `SlewRates angularSlew` | 1500 / 3000 per s | Angular output rate limit |
| `PidGains strafe` | `kP 10` | `strafe` distance (holonomic `moveToPoint` uses `linear`) |
| `SlewRates strafeSlew` | 500 / 1500 per s | Strafe output rate limit |

These defaults are starting points only. Tune them for your robot.

### PidGains

| Field | Default | Meaning |
|---|---|---|
| `kP`, `kI`, `kD` | `0` | Gains, output in speed units |
| `integralZone` | `0` | Integrate only while \|error\| is below this; 0 = always |
| `integralLimit` | `127` | Cap on \|kI × integral\| |
| `resetOnSignFlip` | `true` | Zero the integral when the error changes sign |
| `derivativeSmoothing` | `1.0` | Filter on D; 1 = none |

## Limits

| Field | Default | Meaning |
|---|---|---|
| `defaultTimeoutMs` | `3000` | Timeout when a motion doesn't set one |
| `motionBrake` | `Brake` | Brake after a settled motion |
| `driverBrake` | `Coast` | Brake when the sticks are centered |
| `moveExits`, `turnExits`, `pivotExits` | see [Motions](../tutorials/6-motions.md#how-a-motion-ends) | Exit windows |
| `settleRadiusInches` | `6.0` | `moveToPoint` stops steering inside this |
| `handoffRadiusInches` | `1.0` | `moveToPose` turn starts inside this... |
| `handoffSpeedInchesPerSecond` | `3.0` | ...and below this speed |
| `useCurrentForStall` | `false` | Also detect stalls from motor current |
| `stallCurrentAmps` | `2.0` | Current that counts as stalled |
| `stallMinCommandSpeed` | `20.0` | Only when the command is at least this |
| `throttleCurve`, `turnCurve` | deadzone 3, linear | Driver curves |
| `velocitySmoothing` | `0.5` | Velocity filter |
| `imuDriftLimitDegrees` | `0.05` | Max drift over the startup still check |
| `imuCalibrationTimeoutMs` | `3000` | IMU calibration time limit |
| `batteryWarningPercent` | `30.0` | Low-battery warning |
| `telemetryChannels` | `0` | `TELEMETRY_MOTION`, `TELEMETRY_POSE`, or both with `\|` |

## FieldWalls

Where each wall's inside face is, in your field frame. Used by `resetFromWall()`.

| Field | Default | Meaning |
|---|---|---|
| `leftXInches` | `-72.0` | Left wall (`Wall::Left`) |
| `rightXInches` | `72.0` | Right wall (`Wall::Right`) |
| `backYInches` | `-72.0` | Back wall, driver side (`Wall::Back`) |
| `frontYInches` | `72.0` | Front wall (`Wall::Front`) |

The defaults are a 144 in field with the origin at its center, the same origin the GPS uses. If your autons use another origin, set these (and the GPS offsets) to match.
