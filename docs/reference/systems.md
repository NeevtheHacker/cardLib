# Systems

Optional services around the chassis. None of them run inside the 10 ms control loops.

## Display

```cpp
Display(Chassis& chassis, TestModes& testModes, BlackBox& blackBox);
```

| Member | Description |
|---|---|
| `start()` | Starts the display task |
| `setAutons(std::span<const AutonEntry> autons)` | The selector list |
| `selectedAuton()`, `alliance()`, `mirrored()` | What's selected |
| `runSelectedAuton()` | Call from `autonomous()` |
| `set(size_t line, const char* label, double value)` | A line on the User page |
| `setControllerLine(size_t line, const char* text)` | Controller screen text (15 characters) |
| `rumble(const char* pattern)` | Controller rumble, e.g. `"-."` |

`AutonEntry` is `{const char* name, void (*run)()}`.

## TestModes

```cpp
explicit TestModes(Chassis& chassis);
```

| Member | Description |
|---|---|
| `request(TestKind kind)` | Queue `PortCheck`, `SignCheck` or `TapeBox` (the Tests page does this) |
| `runPortCheck()` | Wheels off the ground: spins each wheel module alone at 10 % for 1 s and reports every motor's direction and speed, plus one terminal line per module saying which way it should push |
| `runPending()` | Runs a queued test. Call from your driver loop. |
| `portCheckReport()`, `signCheckReport()`, `tapeBoxReport()` | Last results |

## CalibrationRoutines

```cpp
explicit CalibrationRoutines(Chassis& chassis);
```

| Member | Description |
|---|---|
| `beginRun()` | Start recording a run |
| `spin(double turns, TurnDirection dir, uint32_t timeoutMs = 60000)` | Spin in place |
| `recordImuRun(double turns)` | IMU scale run |
| `recordPushRun(double tapedInches)` | Straight push run. On a holonomic drive the drive scale comes from forward kinematics over every module |
| `recordSidewaysPushRun(double tapedInches)` | Sideways push run |
| `recordSpinRun()` | Spin run for offsets and track width (track width is skipped on a holonomic drive) |
| `addDistancePoint(size_t sensor, double trueInches)`, `distanceFit(size_t sensor)` | Distance sensor fit |
| `results()` | The measured `Calibration` |
| `printResults()` | Prints copy-paste C++ to the terminal |
| `clear()` | Forget recorded runs |

## Brain screen pages

Auton, Auton options, Pose, Diagnostics, **Sensors**, Tests, Tuner and User. The Sensors page shows every configured sensor as OK, Suspect, Faulted, Wrong (duplicate or invalid port, wrong device, wrong ADI mode) or Missing (nothing found). Diagnostics lists startup problems first: missing ports, "no drive", "modules with no motors", no heading source, IMU problems, and "field-centric ignored: tank drive".

## CalibrationFile

`CalibrationFile::load(path, calibration)` returns `LoadResult::Loaded` when a file was read; keys in the file override the values passed in.

## TerminalLog and BlackBox

| Class | Constructor | Use |
|---|---|---|
| `TerminalLog` | `(TelemetryBuffer&)`, pass `chassis.telemetry()` | `start()`; prints telemetry as CSV |
| `BlackBox` | `(Chassis&)` | Last 64 motions; saved to SD from the Diagnostics page |
