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

## Drive presets

`#include "cardLib/api.hpp"` · from `cardLib/chassis/drive_presets.hpp`

Each preset returns a `DriveConfig` for `config.drive`. They are all `constexpr`, so a drive declared `constexpr` at namespace scope can be checked with `static_assert` before the code is compiled. Usage and worked examples: [Holonomic drives](../tutorials/10-holonomic.md#presets).

| Function | Description |
|---|---|
| `DriveConfig tank(MotorGroup& left, MotorGroup& right, double wheelDiameterInches, Gearing gearing, double trackWidthInches)` | A tank: one module per side at x = −W/2 and +W/2, rolling forward. Also sets `left` and `right`. |
| `DriveConfig asterisk(const AsteriskConfig& config)` | Four 45° corners plus two forward-rolling center wheels. |
| `DriveConfig customDrive(std::initializer_list<WheelModule> modules)` | Any drive: list the modules in the standard order (front-left, front-right, back-left, back-right, center-left, center-right). Modules past 8 are ignored. |
| `SpeedMatchReport speedMatch(const DriveConfig& drive, Cartridge cartridge, MotorType type = MotorType::V5_11W)` | Top forward, strafe and spin speeds, each wheel's forward duty and the overall `mismatch`. Pass the cartridge: a motor group can't be read at compile time. |
| `bool requireSpeedMatchWithin(const SpeedMatchReport& report, double fraction)` | `true` if the faster wheels are cut back by at most `fraction` (e.g. `0.10`). For `static_assert`. |
| `DriveKinematics presetKinematics(const DriveConfig& drive, Cartridge cartridge, MotorType type = MotorType::V5_11W)` | The full kinematics behind `speedMatch`. You won't normally need it. |
| `DriveModule toDriveModule(const WheelModule& module)` | Turns one `WheelModule` into a `DriveModule`. The presets use it; you won't normally call it. |

```cpp
constexpr cardLib::DriveConfig DRIVE = cardLib::asterisk({ /* ... */ });
static_assert(cardLib::requireSpeedMatchWithin(
                  cardLib::speedMatch(DRIVE, cardLib::Cartridge::Blue600), 0.10),
              "corner and center wheels disagree by more than 10%");
```

### WheelModule

| Field | Default | Meaning |
|---|---|---|
| `MotorGroup* motors` | `nullptr` | The motor group driving this wheel |
| `wheelDiameterInches` | **required** | `Omniwheel::` constant or your measured value |
| `Gearing gearing` | **required** | `{motorTeeth, wheelTeeth}` or a ratio |
| `xInches`, `yInches` | `0` | Wheel contact point from the robot center (+x right, +y forward) |
| `rollAngleDegrees` | **required** | Which way a positive command pushes the robot, clockwise from forward |

### AsteriskConfig

| Field | Meaning |
|---|---|
| `std::array<MotorGroup*, 4> corners` | Front-left, front-right, back-left, back-right |
| `cornerWheel`, `Gearing cornerGearing` | Corner wheel diameter and gearing (**required**) |
| `cornerHalfWidthInches`, `cornerHalfLengthInches` | Corner contact point right of and forward of center (**required**) |
| `std::array<MotorGroup*, 2> centers` | Center-left, center-right |
| `centerWheel`, `Gearing centerGearing` | Center wheel diameter and gearing (**required**) |
| `centerHalfWidthInches` | Center contact point right of center (**required**) |

Front-left and back-right roll at +45°, front-right and back-left at −45°.

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
| `stallMinCommandSpeed` | `20.0` | Only when the command, averaged over the driven wheels, is at least this |
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
