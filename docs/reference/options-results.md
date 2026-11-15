# Options and results

## MotionOptions

`struct cardLib::MotionOptions`. Pass with named fields: `{.maxSpeed = 80, .timeoutMs = 1500}`.

| Field | Type | Default | Meaning |
|---|---|---|---|
| `maxSpeed` | `double` | `127` | Output cap, 0 to 127 |
| `minSpeed` | `double` | `0` | Keep at least this speed until within `earlyExit` |
| `earlyExit` | `double` | `0` (off) | End as `Settled` without braking once the error is below this (in or °) |
| `timeoutMs` | `std::optional<uint32_t>` | empty | Empty uses `limits.defaultTimeoutMs` |
| `forwards` | `bool` | `true` | `false` drives backwards, or faces away in `turnToPoint` |
| `direction` | `TurnDirection` | `Shortest` | `Shortest`, `Clockwise`, `CounterClockwise` |
| `async` | `bool` | `false` | Return `Started` at once; use `waitUntilDone()` |
| `allowDegraded` | `bool` | `false` | Run even on degraded odometry |
| `headingDegrees` | `std::optional<double>` | empty | Holonomic `moveToPoint`: the heading to hold. Empty = the heading it started at |
| `tankStyle` | `bool` | `false` | On a holonomic drive, run `moveToPoint` / `moveToPose` the tank way |

## MotionResult

`enum class cardLib::MotionResult`

| Value | Meaning |
|---|---|
| `Started` | Returned by an async motion right away |
| `Settled` | Reached the target (or hit `earlyExit`) |
| `Stalled` | Stopped short and isn't moving |
| `TimedOut` | Ran out of time |
| `Cancelled` | `cancel()`, a new motion, or driver control took over |
| `SensorFault` | A required sensor failed with no fallback, or odometry is degraded and not allowed |
| `NotCalibrated` | Startup validation hasn't passed |
| `Unsupported` | This drive can't make the motion (e.g. `strafe` on a tank). Returned at once; the motion never starts |

If two happen in the same loop, the higher one in this table wins (e.g. `Cancelled` over `TimedOut`).

## Pose and RobotState

```cpp
struct Pose { double xInches; double yInches; double headingDegrees; };

struct RobotState {
  Pose pose;
  double linearVelocityInchesPerSecond;    // along the heading
  double sidewaysVelocityInchesPerSecond;  // to the robot's right
  double angularVelocityDegreesPerSecond;  // clockwise +
  OdometryHealth health;                   // Ok, Degraded, Faulted
  bool wheelHeadingSuspect;                // wheels and IMU disagree (slip or a lifted wheel)
  bool driveSlipSuspect;                   // drive wheels disagree with each other (holonomic only)
};
```

The two suspect flags are warnings only; they never change the pose.

## ResetResult

`enum class cardLib::ResetResult`, returned by `resetFromWall()` and `resetFromGps()`. Anything but `Applied` leaves the pose untouched.

| Value | Meaning |
|---|---|
| `Applied` | The pose was corrected |
| `NotRunning` | `initialize()` hasn't run |
| `NoSensor` | That sensor isn't configured |
| `NoReading` | The sensor gave no reading |
| `OutOfRange` | The reading is outside the sensor's range |
| `NotStopped` | The robot is moving or turning (over 1 in/s or 2°/s) |
| `NotSquare` | The distance sensor is more than 15° from square to the wall |
| `ChangeTooLarge` | The correction is bigger than the limit (4 in) |
| `LowQuality` | The GPS reports a poor fix (only if PROS ever reports a quality) |
| `ErrorTooHigh` | The GPS error estimate is over the limit |

### WallResetOptions and GpsResetOptions

| Field | Default |
|---|---|
| `WallResetOptions::maxAngleFromSquareDegrees` | `15` |
| `WallResetOptions::maxChangeInches` | `4` |
| `WallResetOptions::minRangeInches`, `maxRangeInches` | 20 mm, 2000 mm |
| `GpsResetOptions::resetHeading` | `false` |
| `GpsResetOptions::maxErrorInches` | `1.0` |
| `GpsResetOptions::minQualityPercent` | `95` (unused until PROS reports a quality) |
| `maxSpeedInchesPerSecond`, `maxTurnRateDegreesPerSecond` (both) | `1`, `2` |

## Driver options

| Type | Values |
|---|---|
| `DriveMode` | `RobotCentric` (default), `FieldCentric` |
| `HolonomicOptions` | `fieldCentric`: `std::optional<bool>`, empty = use the drive mode |

## Enums

| Enum | Values |
|---|---|
| `TurnDirection` | `Shortest`, `Clockwise`, `CounterClockwise` |
| `Side` | `Left`, `Right` |
| `Wall` | `Left` (−x), `Right` (+x), `Back` (−y, driver side), `Front` (+y) |
| `LocalOffset` | `{rightInches, forwardInches}`, a point on the robot |
| `BrakeMode` | `Coast`, `Brake`, `Hold` |
| `OdometryHealth` | `Ok`, `Degraded`, `Faulted` |
