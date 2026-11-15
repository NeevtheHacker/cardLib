// arc_odometry.hpp
// Why: the default pose estimator: SPEC section 5 arc odometry with the section 4 fallbacks.
// Owns: the pose, the velocity estimates and the IMU baseline. Only the odometry task calls it.
#pragma once

#include <array>
#include <cstddef>
#include <optional>

#include "cardLib/algorithms/control/drive_kinematics.hpp"
#include "cardLib/algorithms/interfaces/pose_estimator.hpp"
#include "cardLib/algorithms/localization/drive_slip_check.hpp"
#include "cardLib/algorithms/localization/heading_disagreement.hpp"
#include "cardLib/algorithms/math/ema_filter.hpp"

namespace cardLib {

// Built by the chassis from Geometry and Calibration. Offsets are the config numbers exactly:
// vertical wheels by x (right positive), the horizontal wheel by y (front positive).
struct ArcOdometryConfig {
  std::size_t verticalWheelCount = 0;
  std::array<double, MAX_VERTICAL_WHEELS> verticalWheelXInches = {};
  bool hasHorizontalWheel = false;
  double horizontalWheelYInches = 0.0;
  bool hasImu = false;
  double imuScale = 1.0;                   // k_imu
  double effectiveTrackWidthInches = 0.0;  // W_eff, for the drive-encoder fallback
  double velocitySmoothing = 0.5;          // EMA alpha (tune)
  // The drive's wheel rows (only positions and roll angles matter here). When some wheel rolls
  // sideways, the drive-encoder fallback uses least-squares forward kinematics on
  // SensorFrame::driveWheels instead of left/right (SPEC 20.12).
  DriveKinematics driveKinematics = {};
};

class ArcOdometry : public PoseEstimator {
 public:
  explicit ArcOdometry(const ArcOdometryConfig& config);

  void reset(const Pose& startPose) override;
  RobotState update(const SensorFrame& frame) override;

  const RobotState& state() const { return m_state; }

 private:
  // A heading change and whether it came from the expected source.
  struct HeadingChange {
    double radians = 0.0;
    bool known = false;
    bool fallback = false;
  };

  HeadingChange headingChange(const SensorFrame& frame);
  std::optional<double> imuHeadingChange(const SensorFrame& frame);
  std::optional<double> trackingWheelHeadingChange(const SensorFrame& frame) const;
  std::optional<double> driveHeadingChange(const SensorFrame& frame) const;
  std::optional<double> forwardChange(const SensorFrame& frame, double headingRadians);
  std::optional<double> driveForwardChange(const SensorFrame& frame, double headingRadians);
  double sidewaysChange(const SensorFrame& frame, double headingRadians);
  double driveSidewaysChange();
  BodyMotion driveBodyMotion(const SensorFrame& frame) const;
  void integrate(double forwardInches, double sidewaysInches, double headingRadians);
  void updateVelocities(double forwardInches, double sidewaysInches, double headingRadians,
                        double dtSeconds);

  ArcOdometryConfig m_config;
  RobotState m_state;
  EmaFilter m_linearVelocity;
  EmaFilter m_sidewaysVelocity;
  EmaFilter m_angularVelocity;
  bool m_holonomicDrive = false;  // some drive wheel rolls sideways: use forward kinematics
  BodyMotion m_driveMotion = {};  // this loop's forward-kinematics result (holonomic only)
  HeadingDisagreement m_disagreement;
  DriveSlipCheck m_driveSlip;  // fed the forward-kinematics residual (holonomic only)
  double m_lastImuRotationDegrees = 0.0;
  bool m_hasImuBaseline = false;
  bool m_degradedThisLoop = false;
};

}  // namespace cardLib
