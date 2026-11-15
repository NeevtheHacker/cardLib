# 7. Chaining motions

By default each motion stops the robot before the next one starts. That's accurate but slow. cardLib gives you three tools to go faster.

## Running something during a motion: async + waitUntil

```cpp
chassis.moveToPoint(24, 48, {.async = true});   // returns MotionResult::Started at once
chassis.waitUntil(12);                          // wait until 12 in of progress
intake.setMode(Intake::Mode::In);               // start the intake mid-drive
cardLib::MotionResult result = chassis.waitUntilDone();
```

- `waitUntil` counts **inches** for moves and **degrees** for turns. It returns `false` if the motion ended first.
- `waitUntilDone` returns the motion's real result.
- `isMoving()` tells you if a motion is still running, and `cancel()` stops it (it ends as `Cancelled`).

Starting a new motion, or calling `tank`/`arcade`, cancels the one running.

## Flowing into the next motion: minSpeed + earlyExit

```cpp
chassis.moveToPoint(24, 24, {.minSpeed = 60, .earlyExit = 4});
chassis.moveToPoint(48, 30);
```

- `minSpeed` keeps the output at least this high until the robot is within `earlyExit`.
- `earlyExit` ends the motion as `Settled` once the error drops below it (inches or degrees), **without braking**.
- The next motion starts from the speed the last one left off at (the slew limiter carries over), so the robot never drops to zero in between.

Use this for paths through waypoints where you don't need to stop exactly at each one.

## When to use which

| You want | Use |
|---|---|
| Exact stop at each point | Plain motions |
| Do something partway through a move | `async` + `waitUntil` |
| Smooth path through waypoints | `minSpeed` + `earlyExit` |
| Give up early if blocked | Check the result for `Stalled` |
