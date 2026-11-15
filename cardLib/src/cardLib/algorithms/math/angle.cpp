// angle.cpp
// Why: implements the SPEC section 1 angle formulas.
// Owns: nothing beyond the math; no state.
#include "cardLib/algorithms/math/angle.hpp"

#include <cmath>
#include <numbers>

namespace cardLib {

double wrapDegrees(double angleDegrees) {
  return angleDegrees - 360.0 * std::floor((angleDegrees + 180.0) / 360.0); // outputs angle in range of [-180,180) and changes the angle inputed to be the shortest path to reach from 0 and gives the direction - or + counterclockwise or just clockwise
}

double turnErrorDegrees(double targetDegrees, double currentDegrees, TurnDirection direction) {
  const double error = wrapDegrees(targetDegrees - currentDegrees); 
  if (direction == TurnDirection::Clockwise && error < 0.0) {
    return error + 360.0;
  }
  if (direction == TurnDirection::CounterClockwise && error > 0.0) {
    return error - 360.0;
  }
  return error; // using wrapDegrees from a target and current it automaticaly lets robot take turn through shortest possible path. However, if TurnDirection is made to be restrictive then the if statemetns are triggered which bypasses the shortest path by just adding or subtracting 360.
}

double directedTurnErrorDegrees(double targetDegrees, double currentDegrees,
                                TurnDirection& direction) {
  const double shortest = turnErrorDegrees(targetDegrees, currentDegrees);
  const double forced = turnErrorDegrees(targetDegrees, currentDegrees, direction);
  if (forced == shortest) {
    direction = TurnDirection::Shortest;
  }
  return forced; // returns the same as the above kinda the only exception is its changing TurnDirection variable to make sure if the robot overshoots durring PID it doesnt try to go and take a loop if there was a restricitve statement made. Reason this isnt all in one function is just because 
}

double headingToPointDegrees(double fromXInches, double fromYInches, double toXInches,
                             double toYInches) {
  return radiansToDegrees(std::atan2(toXInches - fromXInches, toYInches - fromYInches));
}

double reverseHeadingDegrees(double headingDegrees) { return wrapDegrees(headingDegrees + 180.0); }

double degreesToRadians(double angleDegrees) { return angleDegrees * std::numbers::pi / 180.0; }

double radiansToDegrees(double angleRadians) { return angleRadians * 180.0 / std::numbers::pi; }

}  // namespace cardLib
