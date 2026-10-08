// drive_pipeline.hpp
// Why: runs SPEC section 7 steps 2 to 6 in one place, so every motion and the simulator share it.
// Owns: the linear and angular slew limiters (their memory carries across chained motions).
#pragma once

#include "cardLib/algorithms/control/drive_command.hpp"
#include "cardLib/algorithms/control/slew_limiter.hpp"

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

class DrivePipeline {
 public:
  DrivePipeline(const SlewRates& linearRates, const SlewRates& angularRates);

  // Steps 2 to 6: limit, slew each channel, mix, desaturate, convert to millivolts.
  DriveCommand update(const ChannelRequest& linear, const ChannelRequest& angular,
                      double dtSeconds);

  // Seeds both slew limiters: zeros for a fresh start, the last speeds when chaining.
  void reset(double linearSpeed = 0.0, double angularSpeed = 0.0);

  void setRates(const SlewRates& linearRates, const SlewRates& angularRates);

  // The last slewed speeds, before desaturation.
  double linearSpeed() const { return m_linearSlew.output(); }
  double angularSpeed() const { return m_angularSlew.output(); }

 private:
  SlewLimiter m_linearSlew;
  SlewLimiter m_angularSlew;
};

}  // namespace cardLib
