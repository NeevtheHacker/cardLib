// drive_sim.hpp
// Why: a laptop stand-in for the drivetrain, so whole motions can be tested (SPEC section 13).
// Owns: each side's speed, the true pose (integrated with the real arc odometry) and the
//       encoder totals the robot would read. Test-only.
#pragma once

#include "cardLib/algorithms/control/drive_command.hpp"
#include "cardLib/algorithms/localization/arc_odometry.hpp"
#include "cardLib/algorithms/localization/sensor_frame.hpp"
#include "cardLib/algorithms/math/pose.hpp"

namespace cardLib::sim {

struct DriveSimConfig {
  double wheelDiameterInches = 3.25;
  double wheelRpm = 450.0;
  double trackWidthInches = 12.0;
  double timeConstantSeconds = 0.15;  // tau (tune against real logs later)
};

// Each side: dv/dt = (v_free * V / 12 - v) / tau. A held side stops at once.
class DriveSim {
 public:
  explicit DriveSim(const DriveSimConfig& config, const Pose& start = {});

  void apply(const DriveCommand& command) { m_command = command; }
  void advance(double dtSeconds);

  // Readings since the last call: drive encoder deltas and a perfect IMU.
  SensorFrame takeFrame(double dtSeconds);

  // Makes the left encoder report a jump the wheel never made (slip, a glitch).
  void injectLeftEncoderJump(double inches) { m_leftReportOffsetInches += inches; }

  const Pose& truePose() const { return m_truth.state().pose; }
  double leftSpeedInchesPerSecond() const { return m_leftSpeed; }
  double rightSpeedInchesPerSecond() const { return m_rightSpeed; }
  double linearSpeedInchesPerSecond() const { return (m_leftSpeed + m_rightSpeed) / 2.0; }
  double leftReportedInches() const { return m_leftTotalInches + m_leftReportOffsetInches; }
  double rightReportedInches() const { return m_rightTotalInches; }

 private:
  double nextSpeed(double speed, std::int32_t millivolts, SideMode mode, double dtSeconds) const;

  DriveSimConfig m_config;
  ArcOdometry m_truth;
  DriveCommand m_command;
  double m_leftSpeed = 0.0;
  double m_rightSpeed = 0.0;
  double m_leftTotalInches = 0.0;
  double m_rightTotalInches = 0.0;
  double m_leftReportOffsetInches = 0.0;
  double m_lastLeftReported = 0.0;
  double m_lastRightReported = 0.0;
};

}  // namespace cardLib::sim
