# Your first project

This page walks through the starter project in the order the robot boots. If you installed cardLib into your own project, create the same pieces in your own files.

```text
src/
├── main.cpp                 PROS lifecycle, hands off to the files below
├── robot/
│   ├── ports.hpp            every port number, checked when the code compiles
│   ├── devices.cpp          motors and sensors built from the ports
│   ├── calibration_values.hpp   measured corrections
│   └── chassis_setup.cpp    the cardLib::Chassis and the screen/log services
├── mechanisms/              intake, pneumatics, ...
├── driver/                  button map and driver loop
├── autons/                  one file per auton, plus the selector list
└── screen/                  your own lines on the brain screen
```

## 1. Ports

Put every port in `robot/ports.hpp`. The `static_assert` lines stop the build if a port is out of range or used twice.

```cpp
constexpr std::uint8_t LEFT_FRONT_PORT = 1;
constexpr std::uint8_t LEFT_BACK_PORT = 2;
constexpr std::uint8_t RIGHT_FRONT_PORT = 3;
constexpr std::uint8_t RIGHT_BACK_PORT = 4;
constexpr std::uint8_t IMU_PORT = 16;
constexpr std::uint8_t VERTICAL_TRACKING_PORT = 17;

constexpr std::array<std::uint8_t, 6> SMART_PORTS{
    LEFT_FRONT_PORT, LEFT_BACK_PORT, RIGHT_FRONT_PORT, RIGHT_BACK_PORT,
    IMU_PORT, VERTICAL_TRACKING_PORT};
static_assert(cardLib::smartPortsValid(SMART_PORTS), "a Smart Port is out of range or used twice");
```

Ports are always positive. Reversing is a separate, explicit setting on each motor.

## 2. Devices

`robot/devices.cpp` describes the hardware. Every motor states its port, direction and cartridge.

```cpp
cardLib::MotorGroup leftDrive({
    {.port = LEFT_FRONT_PORT, .reversed = true, .cartridge = cardLib::Cartridge::Blue600},
    {.port = LEFT_BACK_PORT,  .reversed = true, .cartridge = cardLib::Cartridge::Blue600},
}, "left drive");

cardLib::MotorGroup rightDrive({
    {.port = RIGHT_FRONT_PORT, .reversed = false, .cartridge = cardLib::Cartridge::Blue600},
    {.port = RIGHT_BACK_PORT,  .reversed = false, .cartridge = cardLib::Cartridge::Blue600},
}, "right drive");

cardLib::Imu imu(IMU_PORT);
cardLib::RotationSensor verticalTrackingSensor(VERTICAL_TRACKING_PORT, false);

const cardLib::TrackingWheelConfig verticalTrackingWheel{
    .sensor = &verticalTrackingSensor, .diameterInches = cardLib::Omniwheel::NEW_275, .xInches = 0.0};
const cardLib::ImuConfig imuConfig{.sensor = &imu, .orientation = cardLib::ImuOrientation::Flat};
```

The second argument of `MotorGroup` is a name. It shows up in startup messages, so "left drive" is better than "group 1".

## 3. The chassis

`robot/chassis_setup.cpp` builds one `cardLib::Chassis` from a `ChassisConfig`. The config is split into six parts so physical facts, measurements and tuning never get mixed up:

```cpp
cardLib::ChassisConfig makeChassisConfig() {
  cardLib::ChassisConfig config;
  // Which motors and the physical facts. Example values: 3.25 in omnis, 36:48 gears,
  // 12.25 in between the wheel centers. Measure your own.
  config.drive = cardLib::tank(leftDrive, rightDrive, cardLib::Omniwheel::NEW_325, {36, 48}, 12.25);
  config.sensors.verticalWheel1 = &verticalTrackingWheel;          // which sensors
  config.sensors.imu = &imuConfig;
  config.calibration = robotCalibration();                         // measured corrections
  config.gains.linear = {.kP = 10.0, .kD = 0.5};                   // tuning
  config.gains.turn = {.kP = 2.0, .kD = 0.2};
  config.limits.defaultTimeoutMs = 3000;                           // behaviour
  return config;
}

cardLib::Chassis chassis(makeChassisConfig());
```

Declaring the chassis as a global is safe: the constructor only stores the config.

## 4. Starting up

Call `chassis.initialize()` once from `initialize()`, with the robot still. It checks every port, calibrates the IMU (about 3 seconds) and starts the odometry and motion tasks.

```cpp
void initialize() {
  display.start();      // optional: brain screen pages
  chassis.initialize();
}
```

If a required device is missing or the wrong type, the screen and terminal say which port, and motions return `NotCalibrated` until it is fixed.

## 5. Your first test

Lift the wheels off the ground and run the **Port / direction** test from the Tests page on the brain screen (see [Autons and the screen](../tutorials/9-autons-screen.md)). Each drive motor spins briefly; fix any `reversed` flag that's wrong. Then put the robot down and try:

```cpp
void autonomous() {
  chassis.setPose(0, 0, 0);
  chassis.move(24);
  chassis.turn(90);
}
```

The gains above are only a starting point. Next: [Conventions](../tutorials/1-conventions.md).
