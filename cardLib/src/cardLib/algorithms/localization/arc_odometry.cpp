// arc_odometry.cpp
// Why: implements SPEC section 5 (steps 2 to 4) and the section 4 fallbacks and checks.
// Owns: nothing beyond ArcOdometry's own state.
#include "cardLib/algorithms/localization/arc_odometry.hpp"

#include <algorithm>
#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"

namespace cardLib {

namespace {

constexpr double SMALL_ANGLE_RADIANS = 1e-6;

// Shrinks an arc of length 1 to its chord. The small-angle branch is the same formula's limit,
// there only to avoid dividing by zero.
double chordScale(double headingChangeRadians) {
  if (std::abs(headingChangeRadians) < SMALL_ANGLE_RADIANS) {
    return 1.0 - headingChangeRadians * headingChangeRadians / 24.0;
  }
  return 2.0 * std::sin(headingChangeRadians / 2.0) / headingChangeRadians;
}

}  // namespace

ArcOdometry::ArcOdometry(const ArcOdometryConfig& config)
    : m_config(config),
      m_linearVelocity(config.velocitySmoothing),
      m_sidewaysVelocity(config.velocitySmoothing),
      m_angularVelocity(config.velocitySmoothing),
      m_holonomicDrive(config.driveKinematics.forwardWeights().rightKnown) {}

void ArcOdometry::reset(const Pose& startPose) {
  m_state = {};
  m_state.pose = startPose;
  m_linearVelocity.reset();
  m_sidewaysVelocity.reset();
  m_angularVelocity.reset();
  m_disagreement.reset();
  m_driveSlip.reset();
  m_hasImuBaseline = false;
}

RobotState ArcOdometry::update(const SensorFrame& frame) {
  m_degradedThisLoop = false;
  if (m_holonomicDrive) {
    m_driveMotion = driveBodyMotion(frame);
    m_driveSlip.update(m_driveMotion.valid ? m_driveMotion.residualInches : 0.0);
  }
  const HeadingChange heading = headingChange(frame);
  const std::optional<double> forward = forwardChange(frame, heading.radians);
  const double sideways = sidewaysChange(frame, heading.radians);

  integrate(forward.value_or(0.0), sideways, heading.radians);
  updateVelocities(forward.value_or(0.0), sideways, heading.radians, frame.dtSeconds);

  if (!heading.known || !forward.has_value()) {
    m_state.health = OdometryHealth::Faulted;
  } else if (heading.fallback || m_degradedThisLoop) {
    m_state.health = OdometryHealth::Degraded;
  } else {
    m_state.health = OdometryHealth::Ok;
  }
  m_state.wheelHeadingSuspect = m_disagreement.suspect();
  m_state.driveSlipSuspect = m_driveSlip.suspect();
  return m_state;
}

// Step 2. IMU first; without it, two parallel tracking wheels; then drive encoders. Losing the
// source the robot is configured for is a fallback (degraded).
ArcOdometry::HeadingChange ArcOdometry::headingChange(const SensorFrame& frame) {
  const std::optional<double> imu = imuHeadingChange(frame);
  const std::optional<double> trackingWheels = trackingWheelHeadingChange(frame);
  const std::optional<double> drive = driveHeadingChange(frame);
  const std::optional<double> wheels = trackingWheels.has_value() ? trackingWheels : drive;

  if (imu.has_value()) {
    if (wheels.has_value()) {
      m_disagreement.update(radiansToDegrees(*wheels), radiansToDegrees(*imu));
    }
    return {*imu, true, false};
  }
  if (trackingWheels.has_value()) {
    return {*trackingWheels, true, m_config.hasImu};
  }
  if (drive.has_value()) {
    return {*drive, true, true};
  }
  return {};
}

// A faulted IMU hands heading to the wheels. A single missed read is no turn this loop; the
// baseline is kept, so the next good read measures the whole turn since the last one.
std::optional<double> ArcOdometry::imuHeadingChange(const SensorFrame& frame) {
  if (!m_config.hasImu) {
    return std::nullopt;
  }
  if (frame.imuFaulted) {
    m_hasImuBaseline = false;
    return std::nullopt;
  }
  if (!frame.imuReadOk) {
    return 0.0;
  }
  if (!m_hasImuBaseline) {
    m_lastImuRotationDegrees = frame.imuRotationDegrees;
    m_hasImuBaseline = true;
    return 0.0;
  }
  const double changeDegrees =
      m_config.imuScale * (frame.imuRotationDegrees - m_lastImuRotationDegrees);
  m_lastImuRotationDegrees = frame.imuRotationDegrees;
  return degreesToRadians(changeDegrees);
}

// (dL - dR) / (s_R - s_L). Symmetric, so the wheels can be configured in either order.
std::optional<double> ArcOdometry::trackingWheelHeadingChange(const SensorFrame& frame) const {
  const auto& wheels = frame.verticalWheels;
  const auto& offsets = m_config.verticalWheelXInches;
  if (m_config.verticalWheelCount < 2 || !wheels[0].usable || !wheels[1].usable ||
      offsets[1] == offsets[0]) {
    return std::nullopt;
  }
  return (wheels[0].inches - wheels[1].inches) / (offsets[1] - offsets[0]);
}

std::optional<double> ArcOdometry::driveHeadingChange(const SensorFrame& frame) const {
  if (m_holonomicDrive) {
    return m_driveMotion.valid ? std::optional<double>(m_driveMotion.headingRadians)
                               : std::nullopt;
  }
  if (!frame.leftDrive.usable || !frame.rightDrive.usable ||
      m_config.effectiveTrackWidthInches <= 0.0) {
    return std::nullopt;
  }
  return (frame.leftDrive.inches - frame.rightDrive.inches) / m_config.effectiveTrackWidthInches;
}

// Step 3, forward: correct each vertical wheel with its own offset, then average.
std::optional<double> ArcOdometry::forwardChange(const SensorFrame& frame,
                                                 double headingRadians) {
  double sum = 0.0;
  std::size_t used = 0;
  for (std::size_t i = 0; i < m_config.verticalWheelCount && i < MAX_VERTICAL_WHEELS; ++i) {
    if (frame.verticalWheels[i].usable) {
      sum += frame.verticalWheels[i].inches + m_config.verticalWheelXInches[i] * headingRadians;
      ++used;
    }
  }
  if (used > 0) {
    m_degradedThisLoop = m_degradedThisLoop || used < m_config.verticalWheelCount;
    return sum / static_cast<double>(used);
  }
  m_degradedThisLoop = m_degradedThisLoop || m_config.verticalWheelCount > 0;
  return driveForwardChange(frame, headingRadians);
}

// Drive encoders as vertical wheels at x = -W/2 and +W/2.
std::optional<double> ArcOdometry::driveForwardChange(const SensorFrame& frame,
                                                      double headingRadians) {
  if (m_holonomicDrive) {
    return m_driveMotion.valid ? std::optional<double>(m_driveMotion.forwardInches)
                               : std::nullopt;
  }
  const double halfTrack = m_config.effectiveTrackWidthInches / 2.0;
  const double left = frame.leftDrive.inches - halfTrack * headingRadians;
  const double right = frame.rightDrive.inches + halfTrack * headingRadians;
  if (frame.leftDrive.usable && frame.rightDrive.usable) {
    return (left + right) / 2.0;
  }
  m_degradedThisLoop = true;
  if (frame.leftDrive.usable) {
    return left;
  }
  if (frame.rightDrive.usable) {
    return right;
  }
  return std::nullopt;
}

// Step 3, sideways. A tank with no horizontal wheel has no sideways motion. A holonomic drive
// without a working horizontal wheel estimates it from the drive encoders, degraded (SPEC 20.12).
double ArcOdometry::sidewaysChange(const SensorFrame& frame, double headingRadians) {
  if (m_config.hasHorizontalWheel && frame.horizontalWheel.usable) {
    return frame.horizontalWheel.inches - m_config.horizontalWheelYInches * headingRadians;
  }
  if (m_holonomicDrive) {
    m_degradedThisLoop = true;
    return driveSidewaysChange();
  }
  if (m_config.hasHorizontalWheel) {
    m_degradedThisLoop = true;
  }
  return 0.0;
}

double ArcOdometry::driveSidewaysChange() {
  return m_driveMotion.valid && m_driveMotion.rightKnown ? m_driveMotion.rightInches : 0.0;
}

// Least-squares forward kinematics over the drive wheels that are usable this loop.
BodyMotion ArcOdometry::driveBodyMotion(const SensorFrame& frame) const {
  std::array<double, MAX_WHEEL_MODULES> inches = {};
  std::array<bool, MAX_WHEEL_MODULES> usable = {};
  const std::size_t count = std::min(frame.driveWheelCount, MAX_WHEEL_MODULES);
  for (std::size_t i = 0; i < count; ++i) {
    inches[i] = frame.driveWheels[i].inches;
    usable[i] = frame.driveWheels[i].usable;
  }
  return m_config.driveKinematics.bodyMotion(std::span<const double>(inches.data(), count),
                                             std::span<const bool>(usable.data(), count));
}

// Step 4: shrink the arc to its chord and rotate it by the average heading.
void ArcOdometry::integrate(double forwardInches, double sidewaysInches, double headingRadians) {
  const double midHeading = degreesToRadians(m_state.pose.headingDegrees) + headingRadians / 2.0;
  const double scale = chordScale(headingRadians);
  const double sinMid = std::sin(midHeading);
  const double cosMid = std::cos(midHeading);
  m_state.pose.xInches += scale * (sidewaysInches * cosMid + forwardInches * sinMid);
  m_state.pose.yInches += scale * (-sidewaysInches * sinMid + forwardInches * cosMid);
  m_state.pose.headingDegrees += radiansToDegrees(headingRadians);
}

void ArcOdometry::updateVelocities(double forwardInches, double sidewaysInches,
                                   double headingRadians, double dtSeconds) {
  if (dtSeconds <= 0.0) {
    return;
  }
  m_state.linearVelocityInchesPerSecond = m_linearVelocity.update(forwardInches / dtSeconds);
  m_state.sidewaysVelocityInchesPerSecond = m_sidewaysVelocity.update(sidewaysInches / dtSeconds);
  m_state.angularVelocityDegreesPerSecond =
      m_angularVelocity.update(radiansToDegrees(headingRadians) / dtSeconds);
}

}  // namespace cardLib
