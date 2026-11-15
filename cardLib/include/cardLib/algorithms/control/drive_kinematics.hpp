// drive_kinematics.hpp
// Why: one drivetrain model for every drive (SPEC 20). A tank is the case where every wheel
//      rolls forward; asterisk, X-drive and custom drives use the same rows and the same mixer.
// Owns: DriveKinematics: each wheel's row, the robot's top speeds, the forward-kinematics
//       weights (all constexpr when the wheels are), desaturation with rotation priority and
//       generalized pivots.
#pragma once

#include <array>
#include <cstddef>
#include <initializer_list>
#include <span>

#include "cardLib/algorithms/control/drive_command.hpp"
#include "cardLib/algorithms/math/constexpr_math.hpp"
#include "cardLib/algorithms/math/pose.hpp"

namespace cardLib {

// One driven wheel in plain numbers. x right and y forward of the robot's center (inches) at
// the wheel's contact point. rollAngleDegrees: the way a positive command pushes the robot,
// clockwise from forward (SPEC 20.1, 20.3).
struct WheelSpec {
  double xInches = 0.0;
  double yInches = 0.0;
  double rollAngleDegrees = 0.0;
  double topSpeedInchesPerSecond = 0.0;  // s_max = pi * D * wheelRpm / 60
};

// A wheel's surface speed per unit of robot motion: s = strafe * v_x + forward * v_y +
// turn * omega, with v in in/s (right, forward) and omega in rad/s clockwise (SPEC 20.5).
struct WheelRow {
  double strafe = 0.0;
  double forward = 0.0;
  double turn = 0.0;
};

// What a motion or the driver asks of the robot, each on the -127 to 127 scale. 127 forward is
// the robot's top forward speed, and likewise for strafe (right) and turn (clockwise).
struct DriveSpeeds {
  double forward = 0.0;
  double strafe = 0.0;
  double turn = 0.0;
};

// Each wheel's command on the -127 to 127 scale: 127 is that wheel's own top speed. held marks
// the wheels a pivot doesn't move.
struct WheelSpeeds {
  std::array<double, MAX_WHEEL_MODULES> speeds = {};
  std::array<bool, MAX_WHEEL_MODULES> held = {};
  std::size_t count = 0;
};

// The robot's motion over one loop, worked out from how far each driven wheel rolled.
struct BodyMotion {
  double rightInches = 0.0;
  double forwardInches = 0.0;
  double headingRadians = 0.0;  // clockwise positive
  double residualInches = 0.0;  // RMS wheel travel the fit can't explain: wheel slip
  bool rightKnown = false;      // false on a drive that can't move sideways (a tank)
  bool valid = false;           // false if too few usable wheels to tell the motion
};

// Least-squares weights: (v_x, v_y, omega) = weights * s, from the wheels marked usable.
struct ForwardWeights {
  std::array<std::array<double, MAX_WHEEL_MODULES>, 3> weights = {};
  bool rightKnown = false;
  bool valid = false;
};

namespace detail {

constexpr double KINEMATICS_EPSILON = 1e-9;

// (sin b, cos b, sin b * y - cos b * x) for one wheel, before dividing by its top speed.
constexpr WheelRow surfaceRow(const WheelSpec& wheel) {
  const double sinRoll = constexprSinDegrees(wheel.rollAngleDegrees);
  const double cosRoll = constexprCosDegrees(wheel.rollAngleDegrees);
  return {.strafe = sinRoll,
          .forward = cosRoll,
          .turn = sinRoll * wheel.yInches - cosRoll * wheel.xInches};
}

constexpr double rowComponent(const WheelRow& row, std::size_t column) {
  return column == 0 ? row.strafe : (column == 1 ? row.forward : row.turn);
}

using Matrix3 = std::array<std::array<double, 3>, 3>;

// Inverts a 3 x 3 matrix by cofactors. False if it is singular next to its diagonal.
constexpr bool invertMatrix3(const Matrix3& m, Matrix3& inverse) {
  const double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                     m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                     m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  if (constexprAbs(det) <= KINEMATICS_EPSILON * constexprAbs(m[0][0] * m[1][1] * m[2][2])) {
    return false;
  }
  for (std::size_t r = 0; r < 3; ++r) {
    for (std::size_t c = 0; c < 3; ++c) {
      const std::size_t r1 = (c + 1) % 3, r2 = (c + 2) % 3;
      const std::size_t c1 = (r + 1) % 3, c2 = (r + 2) % 3;
      inverse[r][c] = (m[r1][c1] * m[r2][c2] - m[r1][c2] * m[r2][c1]) / det;
    }
  }
  return true;
}

// Solves the 3 x 3 normal equations (A^T A) v = A^T s for the weights, with a column that no
// usable wheel touches pinned to 0 (a tank can't measure sideways motion).
constexpr ForwardWeights solveForwardWeights(const std::array<WheelRow, MAX_WHEEL_MODULES>& rows,
                                             std::size_t count,
                                             const std::array<bool, MAX_WHEEL_MODULES>& usable) {
  std::array<bool, 3> known = {};
  Matrix3 normal = {};
  double scale = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    for (std::size_t r = 0; usable[i] && r < 3; ++r) {
      known[r] = known[r] || constexprAbs(rowComponent(rows[i], r)) > KINEMATICS_EPSILON;
      for (std::size_t c = 0; c < 3; ++c) {
        normal[r][c] += rowComponent(rows[i], r) * rowComponent(rows[i], c);
      }
      scale = normal[r][r] > scale ? normal[r][r] : scale;
    }
  }
  for (std::size_t r = 0; r < 3; ++r) {
    normal[r][r] = known[r] ? normal[r][r] : (scale > 0.0 ? scale : 1.0);
  }
  ForwardWeights result;
  Matrix3 inverse = {};
  if (!known[1] || !known[2] || !invertMatrix3(normal, inverse)) {
    return result;
  }
  for (std::size_t r = 0; r < 3; ++r) {
    for (std::size_t i = 0; i < count; ++i) {
      double weight = 0.0;
      for (std::size_t c = 0; usable[i] && c < 3; ++c) {
        weight += inverse[r][c] * rowComponent(rows[i], c);
      }
      result.weights[r][i] = known[r] ? weight : 0.0;
    }
  }
  result.rightKnown = known[0];
  result.valid = true;
  return result;
}

}  // namespace detail

class DriveKinematics {
 public:
  constexpr DriveKinematics() = default;

  // Up to MAX_WHEEL_MODULES wheels, in the drive's module order; extra wheels are ignored.
  constexpr explicit DriveKinematics(std::span<const WheelSpec> wheels) {
    for (const WheelSpec& spec : wheels) {
      if (m_count == MAX_WHEEL_MODULES) {
        break;
      }
      m_wheels[m_count++] = spec;
    }
    build();
  }
  constexpr DriveKinematics(std::initializer_list<WheelSpec> wheels)
      : DriveKinematics(std::span<const WheelSpec>(wheels.begin(), wheels.size())) {}

  constexpr std::size_t wheelCount() const { return m_count; }
  constexpr const WheelSpec& wheel(std::size_t index) const { return m_wheels[index]; }

  // n_i: the wheel's row divided by its top speed, i.e. the fraction of its top speed per in/s
  // and per rad/s (SPEC 20.7).
  constexpr const WheelRow& normalizedRow(std::size_t index) const { return m_normalized[index]; }

  // The wheel's surface speed per unit of robot motion (SPEC 20.5), before dividing.
  constexpr const WheelRow& surfaceRow(std::size_t index) const { return m_surface[index]; }

  // min over wheels of s_max / |component|; 0 if no wheel can push that way.
  constexpr double maxForwardInchesPerSecond() const { return m_maxForward; }
  constexpr double maxStrafeInchesPerSecond() const { return m_maxStrafe; }
  constexpr double maxSpinRadiansPerSecond() const { return m_maxSpin; }

  // True if some wheel pushes sideways, so strafing and holonomic motions are possible.
  constexpr bool canStrafe() const { return m_maxStrafe > 0.0; }

  // A wheel's share of its top speed when the robot drives forward at full speed (SPEC 20.6).
  constexpr double forwardDuty(std::size_t index) const {
    return constexprAbs(m_scaled[index].forward);
  }

  // False when turning about this robot-local point needs sideways motion the drive can't make
  // (a tank pivoting about a point ahead of or behind its axle line).
  constexpr bool canPivotAbout(const LocalOffset& pivot) const {
    return canStrafe() || constexprAbs(pivot.forwardInches) <= detail::KINEMATICS_EPSILON;
  }

  // The weights for all wheels (computed with the kinematics, constexpr).
  constexpr const ForwardWeights& forwardWeights() const { return m_forward; }

  // SPEC 20.8 on the 127 scale. 1: if any wheel can't make the turn alone, the turn shrinks
  // until the busiest one can. 2: the largest k in [0, 1] keeps every wheel within 127 with
  // k * translation + rotation. 3: each wheel gets k * T_i + R_i.
  WheelSpeeds wheelSpeeds(const DriveSpeeds& request) const;

  // A turn about a robot-local point at turnSpeed (-127 to 127, positive clockwise) (SPEC 20.9).
  // The fastest wheel runs at |turnSpeed|; wheels the pivot doesn't move are marked held.
  WheelSpeeds pivotSpeeds(double turnSpeed, const LocalOffset& pivot) const;

  // Least-squares forward kinematics (SPEC 20.12): the robot's motion from each wheel's travel
  // (inches, in module order). Unusable wheels are left out of the fit.
  BodyMotion bodyMotion(std::span<const double> wheelInches, std::span<const bool> usable) const;

 private:
  constexpr void build() {
    for (std::size_t i = 0; i < m_count; ++i) {
      m_surface[i] = detail::surfaceRow(m_wheels[i]);
    }
    m_maxForward = limitOver(1);
    m_maxStrafe = limitOver(0);
    m_maxSpin = limitOver(2);
    for (std::size_t i = 0; i < m_count; ++i) {
      const double topSpeed = m_wheels[i].topSpeedInchesPerSecond;
      const WheelRow& surface = m_surface[i];
      if (topSpeed <= 0.0) {
        continue;
      }
      m_normalized[i] = {.strafe = surface.strafe / topSpeed,
                         .forward = surface.forward / topSpeed,
                         .turn = surface.turn / topSpeed};
      m_scaled[i] = {.forward = surface.forward * m_maxForward / topSpeed,
                     .strafe = surface.strafe * m_maxStrafe / topSpeed,
                     .turn = surface.turn * m_maxSpin / topSpeed};
    }
    std::array<bool, MAX_WHEEL_MODULES> all = {};
    for (std::size_t i = 0; i < m_count; ++i) {
      all[i] = true;
    }
    m_forward = detail::solveForwardWeights(m_surface, m_count, all);
  }

  // The robot's top speed along one row component: the slowest wheel decides.
  constexpr double limitOver(std::size_t column) const {
    double limit = 0.0;
    for (std::size_t i = 0; i < m_count; ++i) {
      const double component = constexprAbs(detail::rowComponent(m_surface[i], column));
      const double topSpeed = m_wheels[i].topSpeedInchesPerSecond;
      if (component <= detail::KINEMATICS_EPSILON || topSpeed <= 0.0) {
        continue;
      }
      const double allowed = topSpeed / component;
      limit = (limit == 0.0 || allowed < limit) ? allowed : limit;
    }
    return limit;
  }

  std::array<WheelSpec, MAX_WHEEL_MODULES> m_wheels = {};
  std::array<WheelRow, MAX_WHEEL_MODULES> m_surface = {};
  std::array<WheelRow, MAX_WHEEL_MODULES> m_normalized = {};
  // Each wheel's command (on the 127 scale) per unit of DriveSpeeds: n_i times the top speeds.
  std::array<DriveSpeeds, MAX_WHEEL_MODULES> m_scaled = {};
  std::size_t m_count = 0;
  double m_maxForward = 0.0;
  double m_maxStrafe = 0.0;
  double m_maxSpin = 0.0;
  ForwardWeights m_forward = {};
};

// The SPEC 20.6 speed-match numbers, worked out from the kinematics (constexpr).
struct SpeedMatchReport {
  std::size_t wheelCount = 0;
  double maxForwardInchesPerSecond = 0.0;
  double maxStrafeInchesPerSecond = 0.0;
  double maxSpinRadiansPerSecond = 0.0;
  std::array<double, MAX_WHEEL_MODULES> forwardDuty = {};
  // 1 - the smallest forward duty among wheels that push forward at all: how much the faster
  // wheels are cut back to match the slowest.
  double mismatch = 0.0;
};

constexpr SpeedMatchReport speedMatchReport(const DriveKinematics& kinematics) {
  SpeedMatchReport report;
  report.wheelCount = kinematics.wheelCount();
  report.maxForwardInchesPerSecond = kinematics.maxForwardInchesPerSecond();
  report.maxStrafeInchesPerSecond = kinematics.maxStrafeInchesPerSecond();
  report.maxSpinRadiansPerSecond = kinematics.maxSpinRadiansPerSecond();
  double smallestDuty = 1.0;
  for (std::size_t i = 0; i < kinematics.wheelCount(); ++i) {
    report.forwardDuty[i] = kinematics.forwardDuty(i);
    if (report.forwardDuty[i] > detail::KINEMATICS_EPSILON &&
        report.forwardDuty[i] < smallestDuty) {
      smallestDuty = report.forwardDuty[i];
    }
  }
  report.mismatch = 1.0 - smallestDuty;
  return report;
}

// For static_assert in robot code (SPEC 20.7): true if no wheel is cut back by more than this
// fraction, e.g. requireSpeedMatchWithin(report, 0.10).
constexpr bool requireSpeedMatchWithin(const SpeedMatchReport& report, double fraction) {
  return report.mismatch <= fraction;
}

// Two wheels at x = -W/2 and +W/2 rolling forward, left first.
constexpr DriveKinematics tankKinematics(double trackWidthInches, double topSpeedInchesPerSecond) {
  return DriveKinematics{{-trackWidthInches / 2.0, 0.0, 0.0, topSpeedInchesPerSecond},
                         {trackWidthInches / 2.0, 0.0, 0.0, topSpeedInchesPerSecond}};
}

// A tank with track width 1 and top speed 1: its numbers are exact, so the SPEC section 7 mixer
// comes out of it unchanged. What DrivePipeline uses until it's given the real drive.
inline constexpr DriveKinematics UNIT_TANK_KINEMATICS = tankKinematics(1.0, 1.0);

}  // namespace cardLib
