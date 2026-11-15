// drive_kinematics.cpp
// Why: the per-loop parts of SPEC 20: desaturation, pivots and forward kinematics. The rows and
//      top speeds they use were built once, in the header's constexpr constructor.
// Owns: nothing beyond DriveKinematics' own state.
#include "cardLib/algorithms/control/drive_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace cardLib {

namespace {

// Below this fraction of the fastest wheel, a pivot leaves a wheel held instead of driven.
constexpr double PIVOT_HELD_FRACTION = 1e-9;

}  // namespace

WheelSpeeds DriveKinematics::wheelSpeeds(const DriveSpeeds& request) const {
  WheelSpeeds result;
  result.count = m_count;

  double busiestRotation = 0.0;
  for (std::size_t i = 0; i < m_count; ++i) {
    busiestRotation = std::max(busiestRotation, std::abs(m_scaled[i].turn * request.turn));
  }
  const double turn =
      busiestRotation > MAX_SPEED ? request.turn * MAX_SPEED / busiestRotation : request.turn;

  double translationScale = 1.0;
  for (std::size_t i = 0; i < m_count; ++i) {
    const double translation =
        m_scaled[i].forward * request.forward + m_scaled[i].strafe * request.strafe;
    if (translation == 0.0) {
      continue;
    }
    const double rotation = m_scaled[i].turn * turn;
    const double room = MAX_SPEED - std::copysign(1.0, translation) * rotation;
    translationScale = std::min(translationScale, room / std::abs(translation));
  }
  translationScale = std::max(translationScale, 0.0);

  for (std::size_t i = 0; i < m_count; ++i) {
    const double translation =
        m_scaled[i].forward * request.forward + m_scaled[i].strafe * request.strafe;
    result.speeds[i] = translationScale * translation + m_scaled[i].turn * turn;
  }
  return result;
}

// The robot's center moves at omega x (center - pivot): (-forward, right) per rad/s clockwise.
WheelSpeeds DriveKinematics::pivotSpeeds(double turnSpeed, const LocalOffset& pivot) const {
  WheelSpeeds result;
  result.count = m_count;
  const double strafePerTurn = -pivot.forwardInches;
  const double forwardPerTurn = pivot.rightInches;

  std::array<double, MAX_WHEEL_MODULES> fractions = {};
  double fastest = 0.0;
  for (std::size_t i = 0; i < m_count; ++i) {
    const WheelRow& row = m_surface[i];
    const double surfaceSpeed =
        row.strafe * strafePerTurn + row.forward * forwardPerTurn + row.turn;
    const double topSpeed = m_wheels[i].topSpeedInchesPerSecond;
    fractions[i] = topSpeed > 0.0 ? surfaceSpeed / topSpeed : 0.0;
    fastest = std::max(fastest, std::abs(fractions[i]));
  }
  for (std::size_t i = 0; i < m_count; ++i) {
    result.held[i] = std::abs(fractions[i]) <= PIVOT_HELD_FRACTION * fastest;
    result.speeds[i] = result.held[i] ? 0.0 : turnSpeed * fractions[i] / fastest;
  }
  return result;
}

BodyMotion DriveKinematics::bodyMotion(std::span<const double> wheelInches,
                                       std::span<const bool> usable) const {
  BodyMotion motion;
  const std::size_t count = std::min({m_count, wheelInches.size(), usable.size()});
  std::array<bool, MAX_WHEEL_MODULES> used = {};
  bool allUsed = count == m_count;
  for (std::size_t i = 0; i < count; ++i) {
    used[i] = usable[i];
    allUsed = allUsed && usable[i];
  }
  const ForwardWeights fit =
      allUsed ? m_forward : detail::solveForwardWeights(m_surface, count, used);
  if (!fit.valid) {
    return motion;
  }

  double velocity[3] = {};
  for (std::size_t r = 0; r < 3; ++r) {
    for (std::size_t i = 0; i < count; ++i) {
      velocity[r] += used[i] ? fit.weights[r][i] * wheelInches[i] : 0.0;
    }
  }
  double squaredError = 0.0;
  std::size_t usedCount = 0;
  for (std::size_t i = 0; i < count; ++i) {
    if (!used[i]) {
      continue;
    }
    const WheelRow& row = m_surface[i];
    const double predicted =
        row.strafe * velocity[0] + row.forward * velocity[1] + row.turn * velocity[2];
    squaredError += (wheelInches[i] - predicted) * (wheelInches[i] - predicted);
    ++usedCount;
  }
  motion.rightInches = velocity[0];
  motion.forwardInches = velocity[1];
  motion.headingRadians = velocity[2];
  motion.residualInches = std::sqrt(squaredError / static_cast<double>(usedCount));
  motion.rightKnown = fit.rightKnown;
  motion.valid = true;
  return motion;
}

}  // namespace cardLib
