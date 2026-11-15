# 1. Conventions

Every number you give cardLib and every number it gives back follows the same rules. Read this once; everything else assumes it.

## Units

| Quantity | Unit | Example name |
|---|---|---|
| Distance | inches | `distanceInches`, `xInches` |
| Angle and heading | degrees | `headingDegrees` |
| Time | milliseconds | `timeoutMs` |
| Speed you command | −127 to 127 | `maxSpeed = 80` |
| Speed the robot reports | inches per second, degrees per second | `linearVelocityInchesPerSecond` |

Speeds are only turned into motor millivolts (−12000 to 12000) at the very last step, so a speed of 127 always means "full power" no matter what the battery is doing.

## The field frame

cardLib uses the same frame as LemLib:

- **+x** points to the robot's right when it faces heading 0.
- **+y** points forward when it faces heading 0.
- **Heading 0** points along +y. Heading **increases clockwise**: 90 faces +x, 180 faces −y, 270 (or −90) faces −x.
- The pose is the **center** of the robot.

```text
              +y  (heading 0)
               ▲
               │
 heading 270 ──┼──► +x  (heading 90)
               │
         heading 180
```

`setPose(x, y, heading)` decides where (0, 0) is. Most teams put it at the robot's starting spot, or at the field center.

## Headings keep counting

The stored heading does not wrap: after two full clockwise spins it reads 720. Turn targets are compared the short way round, so `turn(0)` from 720 does nothing. If you want a turn to go a particular way, pass a direction:

```cpp
chassis.turn(90, {.direction = cardLib::TurnDirection::CounterClockwise});
```

A turn of exactly 180° goes counterclockwise unless you pick a direction.

## Robot-local vs field

Some values are relative to the robot: **forward** is the way it faces and **right** is its right side. Tracking-wheel and sensor positions are given this way (x to the robot's right, y forward of center). Motion targets in `moveToPoint` and `moveToPose` are field coordinates.

## Signs to remember

| Thing | Positive means |
|---|---|
| `move(d)` | forward (negative goes backward) |
| Turn speed, angular velocity | clockwise |
| A motor command | the direction you set with `reversed` so that positive drives the robot forward |
| Tracking wheel `xInches` | the wheel is right of center |
| Tracking wheel `yInches` | the wheel is forward of center |
