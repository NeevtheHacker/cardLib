// slew_limiter.hpp
// Why: stops the drive output from jumping, with a looser limit for slowing down (SPEC 7).
// Owns: one channel's last output.
#pragma once

namespace cardLib {

// Speed units per second. 0 turns a limit off.
struct SlewRates {
  double accelPerSecond = 0.0;
  double decelPerSecond = 0.0;
};

class SlewLimiter {
 public:
  explicit SlewLimiter(const SlewRates& rates) : m_rates(rates) {}

  // Moves the output toward target by at most one step. A sign flip counts as slowing down and
  // stops at zero for that loop, so the reverse side always starts at the accel rate.
  double update(double target, double dtSeconds);

  // Sets the output directly: 0 for a fresh start, the last output when chaining.
  void reset(double output = 0.0) { m_output = output; }

  void setRates(const SlewRates& rates) { m_rates = rates; }
  double output() const { return m_output; }

 private:
  SlewRates m_rates;
  double m_output = 0.0;
};

}  // namespace cardLib
