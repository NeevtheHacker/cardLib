# 9. Autons and the screen

## One file per auton

In the starter, each auton is its own file in `src/autons/`. To add one:

1. Copy `_new_auton_template.cpp` and rename the file and the function.
2. Declare the function in `auton_list.hpp`.
3. Add one line to the list in `auton_list.cpp`:

```cpp
const cardLib::AutonEntry AUTONS[] = {
    {"Left", leftAuton},
    {"Right", rightAuton},
    {"Skills right", skillsRight},
};
```

Every auton starts by telling odometry where the robot is:

```cpp
void leftAuton() {
  chassis.setPose(0, 0, 0);
  chassis.turn(74);
  chassis.moveToPose(29, 8, 74, {.maxSpeed = 50});
}
```

And `autonomous()` just runs whatever is selected:

```cpp
void autonomous() { display.runSelectedAuton(); }
```

## The display

One task owns the brain screen and the controller screen. Start it before `chassis.initialize()` so it can show startup problems:

```cpp
cardLib::TestModes testModes(chassis);
cardLib::BlackBox blackBox(chassis);
cardLib::Display display(chassis, testModes, blackBox);

void setupRobot() {
  display.setAutons(autonList());
  display.start();
  chassis.initialize();
}
```

**Buttons:** the center button goes to the next page. Left and right do the page's action.

| Page | Shows | Left / right |
|---|---|---|
| Auton | Selected auton | Previous / next auton |
| Auton options | Alliance, mirrored | Toggle alliance / toggle mirror |
| Pose | x, y, heading | — |
| Diagnostics | Startup report, sensor status | — / save black box to SD |
| Tests | Port / direction, sign check, tape box | Pick test / run it |
| Tuner | Live PID gains | Step value down / up (center = next field) |
| User | Your own lines | — |

Your own lines: `display.set(0, "Intake rpm", rpm);`. Controller text: `display.setControllerLine(0, "Auton: Left");` and `display.rumble("-.")`.

## Test modes

Tests requested from the screen run in your driver loop, so call this every loop:

```cpp
testModes.runPending();
```

| Test | Do this | Pass when |
|---|---|---|
| Port / direction | Wheels off the ground | Every motor spins the way the screen says |
| Sign check | Start it, push the robot by hand, run it again | Pushed forward, the vertical wheels and drive read positive; turned clockwise, the IMU does; pushed right, the horizontal wheel does |
| Tape box | Drive a closed loop | The end error is small |

## Terminal log and black box

```cpp
cardLib::TerminalLog terminalLog(chassis.telemetry());
terminalLog.start();
```

prints the telemetry channels you turned on as CSV in `pros terminal`. The black box keeps the last 64 motions in memory; save them to SD from the Diagnostics page after a bad run.
