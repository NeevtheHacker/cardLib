// drive_pipeline.hpp
// Why: runs SPEC section 7 steps 2 to 6 in one place, so every motion and the simulator share it.
//      Since SPEC 20 the mix and desaturation go through the drive's kinematics, per wheel.
// Owns: the linear, strafe and angular slew limiters (their memory carries across chained
//       motions) and the drive kinematics it mixes for.
#pragma once

#include "cardLib/algorithms/control/drive_command.hpp"
#include "cardLib/algorithms/control/drive_kinematics.hpp"
#include "cardLib/algorithms/control/slew_limiter.hpp"
#include "cardLib/algorithms/math/pose.hpp"

namespace cardLib {

struct SpeedLimits {
  double maxSpeed = 127.0;
  double minSpeed = 0.0;        // 0 = off
  double minSpeedCutoff = 0.0;  // minSpeed applies only while |error| > this: earlyExit if
                                // set, otherwise the motion's small exit error
};

// One channel's PID output, the error it came from, and its limits.
struct ChannelRequest {
  double speed = 0.0;
  double error = 0.0;
  SpeedLimits limits = {};
};

// Step 2: clamp to +-maxSpeed, then apply minSpeed with the error's sign.
double limitSpeed(const ChannelRequest& request);

// Wheel speeds to millivolts, with held wheels stopped in Hold. The side fields get the mean of
// the wheels left (x < 0) and right (x > 0) of center.
DriveCommand toDriveCommand(const WheelSpeeds& wheels, const DriveKinematics& kinematics);

// Sets a per-wheel command's side fields: the rounded mean of the wheels left (x < 0) and right
// (x > 0) of center, Held only if every wheel on that side is held. A side with no wheels gets
// 0. For commands built outside the pipeline, e.g. raw per-wheel test voltages.
void summarizeSides(DriveCommand& command, const DriveKinematics& kinematics);

double drivenMillivoltMagnitude(const DriveCommand& command);

class DrivePipeline {
 public:
  // A unit tank (UNIT_TANK_KINEMATICS), with the strafe channel slewed like linear. The real
  // drive is set later with setKinematics().
  DrivePipeline(const SlewRates& linearRates, const SlewRates& angularRates);
  DrivePipeline(const DriveKinematics& kinematics, const SlewRates& linearRates,
                const SlewRates& strafeRates, const SlewRates& angularRates);

  // Steps 2 to 6: limit, slew each channel, mix, desaturate, convert to millivolts. The strafe
  // channel slews back to 0.
  DriveCommand update(const ChannelRequest& linear, const ChannelRequest& angular,
                      double dtSeconds);

  // The same with a strafe channel (positive = right) for holonomic motions (SPEC 20.8).
  DriveCommand updateHolonomic(const ChannelRequest& linear, const ChannelRequest& strafe,
                               const ChannelRequest& angular, double dtSeconds);

  // Pivot turn (SPEC section 8): the turn channel is limited and slewed like angular, then
  // turns about the locked side's wheels, which are held. Positive turn speed turns clockwise.
  // Linear and strafe slew reset to 0.
  DriveCommand updatePivot(const ChannelRequest& turn, Side lockedSide, double dtSeconds);

  // The same about any robot-local point (SPEC 20.9). Check canPivotAbout() first.
  DriveCommand updatePivotAbout(const ChannelRequest& turn, const LocalOffset& pivot,
                                double dtSeconds);

  // Seeds the slew limiters: zeros for a fresh start, the last speeds when chaining.
  void reset(double linearSpeed = 0.0, double angularSpeed = 0.0, double strafeSpeed = 0.0);

  void setRates(const SlewRates& linearRates, const SlewRates& angularRates);
  void setRates(const SlewRates& linearRates, const SlewRates& strafeRates,
                const SlewRates& angularRates);
  void setKinematics(const DriveKinematics& kinematics) { m_kinematics = kinematics; }
  const DriveKinematics& kinematics() const { return m_kinematics; }

  // The last slewed speeds, before desaturation.
  double linearSpeed() const { return m_linearSlew.output(); }
  double strafeSpeed() const { return m_strafeSlew.output(); }
  double angularSpeed() const { return m_angularSlew.output(); }

 private:
  // The point a locked side turns about: the mean position of that side's wheels.
  LocalOffset sidePivot(Side lockedSide) const;

  DriveKinematics m_kinematics;
  SlewLimiter m_linearSlew;
  SlewLimiter m_strafeSlew;
  SlewLimiter m_angularSlew;
};

}  // namespace cardLib
