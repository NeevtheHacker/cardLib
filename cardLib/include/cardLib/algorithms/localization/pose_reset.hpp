// pose_reset.hpp
// Why: snaps the pose to the field on request (SPEC 21): a distance sensor facing a wall sets
//      one coordinate, the GPS sets x, y and optionally heading. Each refuses, leaving the pose
//      as it was, unless the reading can be trusted.
// Owns: the reset math and refusal rules. No state.
#pragma once

#include <optional>

#include "cardLib/algorithms/interfaces/robot_state.hpp"
#include "cardLib/algorithms/localization/sensor_frame.hpp"
#include "cardLib/algorithms/math/pose.hpp"

namespace cardLib {

// Left = -x, Right = +x, Back = -y (the driver's side), Front = +y.
enum class Wall { Left, Right, Back, Front };

// Where each wall's inside face is, in the team's frame. The default is a 144 in field with
// the origin at its center, the same origin the GPS uses; teams with another origin set theirs.
struct FieldWalls {
  double leftXInches = -72.0;
  double rightXInches = 72.0;
  double backYInches = -72.0;
  double frontYInches = 72.0;
};

enum class ResetResult {
  Applied,         // the pose was corrected
  NotRunning,      // initialize() hasn't run
  NoSensor,        // that sensor isn't configured
  NoReading,       // the sensor gave no reading
  OutOfRange,      // the reading is outside the sensor's range
  NotStopped,      // the robot is moving or turning
  NotSquare,       // the sensor is too far from square to the wall
  ChangeTooLarge,  // the correction is bigger than the limit
  LowQuality,      // the GPS reports a poor fix
  ErrorTooHigh,    // the GPS error estimate is over the limit
};

// "Stopped" for a reset: below these speeds (tune).
constexpr double RESET_MAX_SPEED_INCHES_PER_SECOND = 1.0;
constexpr double RESET_MAX_TURN_RATE_DEGREES_PER_SECOND = 2.0;

// SPEC 21.2 limits. The range is the V5 distance sensor's 20 to 2000 mm.
struct WallResetOptions {
  double maxAngleFromSquareDegrees = 15.0;
  double maxChangeInches = 4.0;
  double minRangeInches = 20.0 / 25.4;
  double maxRangeInches = 2000.0 / 25.4;
  double maxSpeedInchesPerSecond = RESET_MAX_SPEED_INCHES_PER_SECOND;
  double maxTurnRateDegreesPerSecond = RESET_MAX_TURN_RATE_DEGREES_PER_SECOND;
};

// A distance sensor's pose on the robot: x right, y forward of center, and the way it points
// relative to the robot's front.
struct DistanceMount {
  double xInches = 0.0;
  double yInches = 0.0;
  double headingDegrees = 0.0;
};

// The pose to use (the corrected one only when result is Applied) and the size of the change.
struct PoseResetDecision {
  ResetResult result = ResetResult::NoReading;
  Pose pose = {};
  double changeInches = 0.0;
};

// SPEC 21.2. The reading is already corrected by the sensor's gain and offset (section 6).
// Sensor distance from the wall along the field axis = reading * cos(beam - wall direction);
// the robot's center is that far in from the wall, less the sensor's offset on the field.
// Only the wall's coordinate changes; the other coordinate and the heading never do.
PoseResetDecision wallReset(const RobotState& state, const DistanceMount& mount,
                            const DistanceReading& reading, Wall wall, const FieldWalls& walls,
                            const WallResetOptions& options);

// Where the GPS sits on the robot, and how its field frame maps onto the team's.
struct GpsMount {
  double xInches = 0.0;             // right of the robot's center
  double yInches = 0.0;             // forward of it
  double headingDegrees = 0.0;      // the way the sensor faces; 180 for the usual rear mount
  double fieldXOffsetInches = 0.0;  // the team's frame = the GPS frame turned, then shifted
  double fieldYOffsetInches = 0.0;
  double fieldRotationDegrees = 0.0;  // clockwise
};

// The GPS sensor's own position (inches, GPS frame) and heading, as it reports them.
struct GpsSensorPose {
  double xInches = 0.0;
  double yInches = 0.0;
  double headingDegrees = 0.0;
};

// The robot's center pose in the team's frame from what the GPS reports.
Pose robotPoseFromGps(const GpsSensorPose& sensor, const GpsMount& mount);

// A GPS fix, converted to the robot's center in the team's frame.
struct GpsReading {
  Pose robotPose = {};
  double errorInches = 0.0;
  std::optional<double> qualityPercent = std::nullopt;  // empty when the platform can't say
  bool valid = false;
};

// SPEC 21.3 limits. minQualityPercent only applies when the GPS reports a quality.
struct GpsResetOptions {
  bool resetHeading = false;
  double minQualityPercent = 95.0;
  double maxErrorInches = 1.0;  // (tune)
  double maxSpeedInchesPerSecond = RESET_MAX_SPEED_INCHES_PER_SECOND;
  double maxTurnRateDegreesPerSecond = RESET_MAX_TURN_RATE_DEGREES_PER_SECOND;
};

// SPEC 21.3 basic reset: x and y, and the heading if asked (kept continuous with the current
// one, so a heading of 725 stays near 725).
PoseResetDecision gpsReset(const RobotState& state, const GpsReading& reading,
                           const GpsResetOptions& options);

}  // namespace cardLib
