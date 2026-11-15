# 6. Motions

Every motion blocks until it finishes and returns a [`MotionResult`](../reference/options-results.md#motionresult). Every motion also takes `MotionOptions` as its last argument, written with named fields:

```cpp
chassis.moveToPoint(24, 36, {.maxSpeed = 80, .timeoutMs = 2000});
```

Anything you leave out keeps its default. A motion with no `timeoutMs` uses `limits.defaultTimeoutMs`.

## move

```cpp
chassis.move(24);                       // 24 in forward, holding the starting heading
chassis.move(-12);                      // 12 in backward
chassis.move(24, {.maxSpeed = 60});
```

Relative to where the robot is when it starts. Sideways drift doesn't count as progress.

## turn

```cpp
chassis.turn(90);    // face heading 90 (field), the short way round
chassis.turn(-45, {.direction = cardLib::TurnDirection::Clockwise});
```

The target is an absolute field heading, not "turn by".

## turnToPoint

```cpp
chassis.turnToPoint(24, 48);                       // face the point
chassis.turnToPoint(24, 48, {.forwards = false});  // face away from it (back toward it)
```

## pivotTurn

```cpp
chassis.pivotTurn(90, cardLib::Side::Left);   // left side stays still, right side drives
```

Turns about one locked side. The locked side holds with brake mode Hold. Useful against walls and goals.

## moveToPoint

```cpp
chassis.moveToPoint(24, 48);
chassis.moveToPoint(0, 0, {.forwards = false});   // drive there backwards
```

Drives toward the point while steering toward it. Inside `limits.settleRadiusInches` (6 in by default) it stops steering and only corrects distance, so it doesn't spin around the point at the end.

## moveToPose

```cpp
chassis.moveToPose(48, 24, 90);   // end at (48, 24) facing 90
```

Two phases: drive to the point, then turn in place to the heading once the robot is within `limits.handoffRadiusInches` (1 in) and slower than `limits.handoffSpeedInchesPerSecond` (3 in/s). One timeout covers both phases.

## How a motion ends

A motion is **Settled** when the error stays inside a small window for a short time (or a looser window for a longer time). It's **Stalled** if it stops moving while still far away, **TimedOut** if it runs out of time. The windows are in `limits.moveExits`, `turnExits` and `pivotExits`:

| | Small window | Large window | Stall |
|---|---|---|---|
| Moves | 1 in for 100 ms | 3 in for 500 ms | under 1 in/s for 300 ms |
| Turns | 1° for 100 ms | 3° for 500 ms | under 2.5°/s for 300 ms |
| Pivots | 1.5° for 100 ms | 4° for 500 ms | under 2.5°/s for 300 ms |

## Using the result

```cpp
if (chassis.moveToPoint(24, 48) != cardLib::MotionResult::Settled) {
  chassis.moveToPoint(0, 0, {.forwards = false});   // back off and skip this goal
}
```

## Speed limits

`maxSpeed` caps the output (0 to 127). `minSpeed` keeps the robot moving until it's close; it's for chaining, see the next page.
