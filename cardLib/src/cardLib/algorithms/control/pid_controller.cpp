// pid_controller.cpp
// Why: implements the SPEC section 7 discrete PID.
// Owns: nothing beyond PidController's own state.
#include "cardLib/algorithms/control/pid_controller.hpp"

#include <algorithm>
#include <cmath>

#include "cardLib/algorithms/math/angle.hpp"
#include "cardLib/algorithms/math/loop_timing.hpp"

namespace cardLib {

namespace {

bool signFlipped(double error, double previousError) {
  return (error > 0.0 && previousError < 0.0) || (error < 0.0 && previousError > 0.0);
}

}  // namespace

PidController::PidController(const PidGains& gains, ErrorKind kind)
    : m_gains(gains), m_kind(kind) {}

double PidController::update(double error, double dtSeconds) {
  const bool overran = isLoopOverrun(dtSeconds);
  updateIntegral(error, dtSeconds, overran);
  updateDerivative(error, dtSeconds, overran);
  m_previousError = error;
  m_hasPrevious = true;
  m_lastOverran = overran;
  return m_gains.kP * error + m_gains.kI * m_integral + m_gains.kD * m_derivative;
}

void PidController::reset() {
  m_integral = 0.0;
  m_derivative = 0.0;
  m_previousError = 0.0;
  m_hasPrevious = false;
  m_lastOverran = false;
}

void PidController::updateIntegral(double error, double dtSeconds, bool overran) {
  const bool inZone = m_gains.integralZone == 0.0 || std::abs(error) < m_gains.integralZone;
  if (m_gains.resetOnSignFlip && m_hasPrevious && signFlipped(error, m_previousError)) {
    m_integral = 0.0;
  } else if (inZone && !overran) {
    m_integral += error * dtSeconds;
  }

  if (m_gains.kI != 0.0) {
    const double integralCap = m_gains.integralLimit / std::abs(m_gains.kI);
    m_integral = std::clamp(m_integral, -integralCap, integralCap);
  }
}

void PidController::updateDerivative(double error, double dtSeconds, bool overran) {
  if (!m_hasPrevious || overran) {
    m_derivative = 0.0;
    return;
  }
  const double change = m_kind == ErrorKind::Angle ? wrapDegrees(error - m_previousError)
                                                   : error - m_previousError;
  const double beta = m_gains.derivativeSmoothing;
  m_derivative = beta * (change / dtSeconds) + (1.0 - beta) * m_derivative;
}

}  // namespace cardLib
