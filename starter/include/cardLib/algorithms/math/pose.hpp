// pose.hpp
// Why: the robot pose and the only two robot-local <-> field formulas, so a sign is wrong once.
// Owns: Pose, Point, LocalOffset and the SPEC section 1 transforms.
#pragma once

namespace cardLib {

// Where the robot's center is on the field and which way it faces.
// Heading: 0 = +y, clockwise positive, kept continuous (720 after two clockwise spins).
struct Pose {
  double xInches = 0.0;
  double yInches = 0.0;
  double headingDegrees = 0.0;
};

// A point on the field.
struct Point {
  double xInches = 0.0;
  double yInches = 0.0;
};

// A point relative to the robot: to its right and ahead of it.
struct LocalOffset {
  double rightInches = 0.0;
  double forwardInches = 0.0;
};

// Where a point on the robot is on the field.
Point localToField(const Pose& pose, const LocalOffset& local);

// How far ahead of and to the right of the robot a field point is.
LocalOffset fieldToLocal(const Pose& pose, const Point& point);

}  // namespace cardLib
