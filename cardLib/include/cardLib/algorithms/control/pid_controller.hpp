// pid_controller.hpp
// Why: one PID class for the drive and every mechanism, with the SPEC section 7 guards.
// Owns: the integral, the last error and the filtered derivative.
#pragma once

namespace cardLib {

// Error in inches or degrees; output on the 0 to 127 speed scale.
struct PidGains {
  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;
  double integralZone = 0.0;           // integrate only while |e| < zone; 0 = always
  double integralLimit = 127.0;        // cap on |kI * I|, speed units
  bool resetOnSignFlip = true;         // zero the integral when the error changes sign
  double derivativeSmoothing = 1.0;    // beta: 1 = no filter
};

// Distances subtract errors directly; angles wrap the change so +179 -> -179 is 2 degrees.
enum class ErrorKind { Distance, Angle };

class PidController {
 public:
  explicit PidController(const PidGains& gains, ErrorKind kind = ErrorKind::Distance);

  // One control step. dtSeconds is the measured loop time.
  double update(double error, double dtSeconds);

  // Forget the integral and the last error, so the next update has no D kick.
  void reset();

  void setGains(const PidGains& gains) { m_gains = gains; }
  const PidGains& gains() const { return m_gains; }

  // True if the last update's dt was an overrun (SPEC section 7) and D/I were skipped.
  bool lastUpdateOverran() const { return m_lastOverran; }

 private:
  void updateIntegral(double error, double dtSeconds, bool overran);
  void updateDerivative(double error, double dtSeconds, bool overran);

  PidGains m_gains;
  ErrorKind m_kind;
  double m_integral = 0.0;
  double m_derivative = 0.0;
  double m_previousError = 0.0;
  bool m_hasPrevious = false;
  bool m_lastOverran = false;
};

}  // namespace cardLib
