# Hardware

Thin wrappers over PROS devices. Every read returns an empty `std::optional` when the device doesn't answer, so a failure is never mistaken for a reading.

## MotorGroup

```cpp
MotorGroup(std::initializer_list<MotorConfig> motors, const char* role = "motor group");
```

| Member | Description |
|---|---|
| `moveMillivolts(int32_t mv)` | −12000 to 12000 |
| `stop(BrakeMode mode)` | Stop with a brake mode |

### MotorConfig

| Field | Default | Meaning |
|---|---|---|
| `port` | `0` | 1 to 21, always positive |
| `reversed` | `false` | Set so positive drives the robot forward |
| `cartridge` | `Blue600` | `Red100`, `Green200`, `Blue600` |
| `type` | `V5_11W` | `V5_11W` or `Exp5_5W` |

A 5.5 W motor always runs at 200 rpm whatever cartridge is set; cardLib treats it as `Green200`.

## Wheels and gearing

`cardLib::Omniwheel::` holds measured rolling diameters (from LemLib's table): `NEW_2` 2.125, `NEW_275`/`OLD_275` 2.75, `NEW_275_HALF` 2.744, `OLD_275_HALF` 2.74, `NEW_325`/`OLD_325` 3.25, `NEW_325_HALF`/`OLD_325_HALF` 3.246, `NEW_4` 4.00, `OLD_4` 4.18, `NEW_4_HALF` 3.995, `OLD_4_HALF` 4.175. A plain number (your push-test value) works anywhere a constant does.

`cardLib::Gearing` is external gearing: `{motorTeeth, wheelTeeth}` such as `{36, 48}` (600 → 450 rpm), or a plain ratio such as `0.75`. It has no default. `wheelRpm(motorRpm, gearing)` and `motorRpm(MotorConfig)` are `constexpr`.

## Sensors

| Class | Constructor | Reads |
|---|---|---|
| `RotationSensor` | `(uint8_t port, bool reversed)` | Rotation (as an `Encoder`) |
| `AdiEncoder` | `(uint8_t topPort, uint8_t bottomPort, bool reversed)` | Rotation, 360 ticks per turn |
| `Imu` | `(uint8_t port)` | `readRotationDegrees()`, `readGyroRateDegreesPerSecond()` |
| `DistanceSensor` | `(uint8_t port)` | `readInches()`, `readConfidence()` |
| `GpsSensor` | `(uint8_t port)` | `read()`: position (inches, field center = 0, 0), heading (clockwise) and error estimate |
| `AdiDigitalOut` | `(uint8_t port, bool initialState = false)` | `set(bool)`, `toggle()`, e.g. pneumatics |

`Encoder` is the shared interface for anything that measures rotation, so a tracking wheel can use a rotation sensor, an ADI encoder or a motor.

## Configs

| Struct | Fields |
|---|---|
| `TrackingWheelConfig` | `sensor`, `diameterInches` (**required**), `gearRatio` (1), `xInches`, `yInches` |
| `ImuConfig` | `sensor`, `orientation` (`Flat`, `UpsideDown`, `XUp`, `XDown`, `YUp`, `YDown`), optional `xInches`, `yInches` |
| `DistanceSensorConfig` | `sensor`, `xInches`, `yInches`, `headingDegrees`, `gain` (1), `offsetInches` (0) |
| `GpsConfig` | `sensor`, `xInches`, `yInches`, `headingDegrees` (the way it faces; 180 for a rear mount), `fieldXOffsetInches`, `fieldYOffsetInches`, `fieldRotationDegrees` (all 0) |

## Port checks

```cpp
static_assert(cardLib::smartPortsValid(SMART_PORTS), "...");   // range + duplicates
static_assert(cardLib::adiPortsValid(ADI_PORTS), "...");       // 'A' to 'H' + duplicates
```

At startup `chassis.initialize()` also checks that every configured Smart Port has the right kind of device plugged in.
