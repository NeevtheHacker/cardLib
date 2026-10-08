// angle.hpp
// Why: one place for angle wrap, turn error and heading-to-point, so a sign is only wrong once.
// Owns: the SPEC section 1 angle formulas. Degrees in, degrees out; radians only in trig.
#pragma once

namespace cardLib {

// Which way a turn may go. Shortest is the default everywhere.
enum class TurnDirection { Shortest, Clockwise, CounterClockwise };

// Wraps any angle into [-180, 180). Exactly +-180 becomes -180.
double wrapDegrees(double angleDegrees);

// Signed error from current to target heading. Positive means turn clockwise.
double turnErrorDegrees(double targetDegrees, double currentDegrees,
                        TurnDirection direction = TurnDirection::Shortest);

// Heading that faces (toX, toY) from (fromX, fromY): 0 = +y, clockwise positive.
double headingToPointDegrees(double fromXInches, double fromYInches, double toXInches,
                             double toYInches);

// The same heading turned around, for driving backwards.
double reverseHeadingDegrees(double headingDegrees);

double degreesToRadians(double angleDegrees);
double radiansToDegrees(double angleRadians);

}  // namespace cardLib
