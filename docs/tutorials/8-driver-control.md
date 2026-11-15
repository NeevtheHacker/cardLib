# 8. Driver control

```cpp
void driverControl() {
  while (true) {
    chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                   master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    pros::delay(10);
  }
}
```

| Call | Sticks |
|---|---|
| `chassis.tank(left, right)` | each stick drives its side |
| `chassis.arcade(forward, turn)` | one value for forward, one for turning |

Values are −127 to 127, straight from `get_analog`. Calling either one cancels any running motion, so a driver can always take over.

## Curves

Small stick movements are hard to control at full sensitivity. cardLib applies a **deadzone** (ignore tiny inputs) and an **expo curve** (gentle near the center, full power at the edges). Throttle and turn have separate curves:

```cpp
config.limits.throttleCurve = {.deadzone = 3.0, .curveGain = 3.0};
config.limits.turnCurve     = {.deadzone = 3.0, .curveGain = 5.0};
```

`curveGain` 0 is linear; bigger values are gentler at low stick. Full stick is always full power.

In `holonomic()`, forward and strafe both use the throttle curve. Robot-centric curves each stick axis on its own. Field-centric curves the length of the stick's push (a full push in any direction is full power), then turns it by the heading, so the feel is the same whichever way the robot faces.

In arcade, if forward + turn would go over 127, forward is reduced so the turn is never lost.

## Brake modes

```cpp
chassis.setDriverBrake(cardLib::BrakeMode::Hold);   // e.g. on a button, to resist pushing
chassis.setDriverBrake(cardLib::BrakeMode::Coast);
```

`chassis.driverBrake()` returns the current mode, so a toggle is one line.

## Robot-centric and field-centric

**Robot-centric** driving (what tank and arcade do) means the stick moves the robot relative to the way it faces. **Field-centric** means stick forward always moves the robot away from the driver, whichever way it faces. Field-centric only makes sense for drives that can strafe, so it arrives with holonomic drives in 0.2, defaulting to robot-centric with a switch. See [Holonomic drives](10-holonomic.md#driver-control).
