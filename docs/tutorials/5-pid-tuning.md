# 5. Tuning PID

cardLib's motions use PID controllers. You tune four of them:

| Gains | Used by |
|---|---|
| `gains.linear` | Distance in `move` and `moveToPoint` |
| `gains.heading` | Holding the heading straight during moves |
| `gains.turn` | `turn`, `turnToPoint`, the final turn of `moveToPose` |
| `gains.pivot` | `pivotTurn` |

Each is a `PidGains`:

```cpp
config.gains.linear = {.kP = 10.0, .kI = 0.0, .kD = 0.5};
config.gains.turn   = {.kP = 2.0,  .kD = 0.2};
```

The output is in speed units (−127 to 127). For `linear`, the error is in inches; for turns, degrees. So `kP = 10` means "10 speed units per inch of error".

## What each term does

- **kP** pushes toward the target, harder the further away you are. Too low: slow and stops short. Too high: overshoots and oscillates.
- **kD** brakes as you approach, based on how fast the error is shrinking. It removes overshoot. Too high: jittery or sluggish.
- **kI** builds up over time to push through a small leftover error. Usually not needed for drive motions. If you use it, set `integralZone` so it only builds near the target.

## The procedure

Tune turns first (they're quick to repeat), then linear.

1. Set kI and kD to 0. Start kP low.
2. Raise kP until the robot overshoots a little and wobbles once or twice before settling.
3. Raise kD until the overshoot is gone.
4. If it now stops short, nudge kP up and repeat step 3.
5. Test at several distances or angles (e.g. 6, 24, 48 in; 15, 90, 180°). One gain set must work for all of them.

## Tuning without re-uploading

The **Tuner** page on the brain screen changes gains live. The center button moves between fields, left and right step the value down and up. Run a motion, adjust, run it again, then copy the final values into `chassis_setup.cpp`. From code:

```cpp
cardLib::Gains gains = chassis.gains();
gains.turn.kD = 0.25;
chassis.setGains(gains);   // takes effect at the next motion loop
```

## Seeing what happened

Turn on telemetry while tuning and open `pros terminal`:

```cpp
config.limits.telemetryChannels = cardLib::TELEMETRY_MOTION;   // target, measured, output
```

The terminal prints CSV you can paste into a spreadsheet and graph. Set it back to 0 for matches.

## Slew

Slew limits how fast the output can rise, so the robot doesn't wheelie or slip at the start of a move. Slowing down is allowed to be faster than speeding up.

```cpp
config.gains.linearSlew  = {.accelPerSecond = 500.0,  .decelPerSecond = 1500.0};
config.gains.angularSlew = {.accelPerSecond = 1500.0, .decelPerSecond = 3000.0};
```

500 per second means full speed (127) takes about a quarter second. Set a value to 0 to turn that limit off.
