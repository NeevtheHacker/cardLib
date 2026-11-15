# 3. Sensors

cardLib tracks position from whatever sensors you give it. More sensors mean better accuracy and more fallbacks, but only the drive motors are strictly required.

## What each sensor does

| Sensor | Gives odometry | Required? |
|---|---|---|
| Drive motor encoders | Forward distance; heading if nothing better | Always there |
| Vertical tracking wheel(s) | Forward distance that doesn't slip with the drive | Optional, strongly recommended |
| Horizontal tracking wheel | Sideways distance (pushes, collisions, drift) | Optional |
| IMU | Heading | Optional, strongly recommended |
| Distance sensors | Snap x or y to a wall, on request with the robot stopped | Optional |
| GPS sensor | Snap x, y and heading to the field, on request with the robot stopped | Optional |

Heading comes from the best source available, in this order: **IMU → two parallel tracking wheels → drive encoders**. If the IMU fails mid-match, cardLib switches source and marks odometry as `Degraded`.

## Tracking wheels

```cpp
cardLib::RotationSensor verticalSensor(17, false);   // port, reversed
cardLib::RotationSensor horizontalSensor(18, false);

// Example positions; measure your own.
const cardLib::TrackingWheelConfig verticalTrackingWheel{
    .sensor = &verticalSensor, .diameterInches = cardLib::Omniwheel::NEW_275, .xInches = 0.0};
const cardLib::TrackingWheelConfig horizontalTrackingWheel{
    .sensor = &horizontalSensor, .diameterInches = cardLib::Omniwheel::NEW_275, .yInches = -1.5};

config.sensors.verticalWheel1 = &verticalTrackingWheel;
config.sensors.horizontalWheel = &horizontalTrackingWheel;
```

- **Vertical** wheels roll forward/back. Give their `xInches`: how far right of center (negative = left).
- **Horizontal** wheels roll sideways. Give their `yInches`: how far forward of center (negative = behind).
- `diameterInches` is required: an `Omniwheel::` constant to start, then your push-test value after calibration.
- `gearRatio` is wheel turns per sensor turn; leave it at 1 when the sensor is on the wheel's shaft.
- A 3-wire encoder works too: `cardLib::AdiEncoder enc('A', 'B', false);`

**Mounting tips:** keep the wheel near the center line it measures along, mount it rigidly and square to the frame, and use light spring or rubber-band pressure so it never lifts. Measure offsets carefully; calibration then corrects the last bit.

## IMU

```cpp
cardLib::Imu imu(16);
const cardLib::ImuConfig imuConfig{.sensor = &imu, .orientation = cardLib::ImuOrientation::Flat};
config.sensors.imu = &imuConfig;
```

`orientation` says how the IMU is mounted (`Flat` = logo up). At startup cardLib compares it with what the IMU detects and reports a mismatch. The IMU's position doesn't matter for heading.

At startup the robot must sit still while the IMU calibrates. cardLib then measures drift for one second and warns if it's too high (`limits.imuDriftLimitDegrees`).

## Distance sensors

```cpp
cardLib::DistanceSensor backDistance(10);
const cardLib::DistanceSensorConfig backDistanceConfig{
    .sensor = &backDistance, .xInches = 0.0, .yInches = -6.5, .headingDegrees = 180.0};
config.sensors.distanceSensors[0] = &backDistanceConfig;
```

Give each one its position and which way it points relative to the robot's front (example values above). `gain` and `offsetInches` come from calibration: the corrected distance is `gain · reading + offset`. Only add sensors once you know where they're mounted; leaving them out never breaks anything else.

Back, left and right (or front, left and right) is a good layout: two walls let you reset both x and y in a corner. With the robot stopped and the sensor facing a wall:

```cpp
cardLib::ResetResult result = chassis.resetFromWall(0, cardLib::Wall::Back);  // sensor 0, back wall
```

It changes only that wall's coordinate (y for `Back`/`Front`, x for `Left`/`Right`), and only when the robot is stopped, the sensor is within 15° of square, the reading is in range and the change is under 4 in. Otherwise it returns why (`NotStopped`, `NotSquare`, `ChangeTooLarge`, ...) and the pose is untouched. Wall positions come from `config.field` (default: a 144 in field centered on the origin).

## GPS sensor

```cpp
cardLib::GpsSensor gps(9);
const cardLib::GpsConfig gpsConfig{
    .sensor = &gps, .xInches = 0.0, .yInches = -7.0, .headingDegrees = 180.0};  // example rear mount
config.sensors.gps = &gpsConfig;
```

Mount it at the back facing backward, level with the field code strip (about 10.5 in up) and not tilted. It can't see within about 13.5 in of a wall and works best about 20 in clear. Give its position and the way it faces; `fieldXOffsetInches`, `fieldYOffsetInches` and `fieldRotationDegrees` move its field frame onto yours if your origin isn't the field center.

```cpp
chassis.resetFromGps();                        // x and y
chassis.resetFromGps({.resetHeading = true});  // x, y and heading
```

It only applies when the robot is stopped and the GPS error estimate is under 1 in. PROS doesn't report a GPS quality number yet, so the error estimate is the check.

!!! warning "Check the GPS frame on the field first"
    cardLib assumes the GPS reports +x right, +y forward and heading clockwise, and does the mount offset itself. Put the robot at two taped points facing two headings and check `resetFromGps` lands on them before relying on it in an auton.

## The Sensors page

The brain's Sensors page lists every configured sensor as **OK**, **Suspect** (answering, but this reading is doubtful: no wall in range, a poor GPS fix, slipping drive wheels), **Faulted** (stopped answering mid-run) or **Missing** (not found at startup).

## When a sensor fails

Every reading is checked each loop. A reading that would need the robot to move faster than it can is thrown away (it never becomes motion). Three in a row marks that sensor as faulted, and odometry switches to the next source. Precision motions refuse to run on degraded odometry unless you allow it:

```cpp
chassis.setAllowDegraded(true);                 // for the whole match
chassis.moveToPoint(24, 0, {.allowDegraded = true});  // or for one motion
```
