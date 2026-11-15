// hardware_config.hpp
// Why: the robot's physical description, one row per device (SPEC sections 3 and 17).
// Owns: plain config structs and enums. No behavior.
#pragma once

#include <cstdint>

namespace cardLib {

class Encoder;
class Imu;
class DistanceSensor;
class GpsSensor;

// Must match the cartridge physically installed; never inferred from a target RPM.
enum class Cartridge { Red100, Green200, Blue600 };

// 11 W and 5.5 W motors have different torque and current limits.
enum class MotorType { V5_11W, Exp5_5W };

enum class BrakeMode { Coast, Brake, Hold };

// Required devices block motions when missing; optional ones only show a warning.
enum class Requirement { Required, Optional };

// How the IMU is mounted. Flat = VEX logo up. The IMU reports heading about gravity either way;
// this is checked against what the IMU itself detects at startup.
enum class ImuOrientation { Flat, UpsideDown, XUp, XDown, YUp, YDown };

struct MotorConfig {
  std::uint8_t port = 0;  // 1 to 21
  bool reversed = false;  // explicit, never guessed
  Cartridge cartridge = Cartridge::Blue600;
  MotorType type = MotorType::V5_11W;
};

// A tracking wheel: its sensor and where it sits, x to the robot's right and y forward of the
// center. Vertical wheels use x, the horizontal wheel uses y (SPEC section 5, step 3).
struct TrackingWheelConfig {
  Encoder* sensor = nullptr;
  double diameterInches = 2.75;
  double gearRatio = 1.0;  // wheel turns per sensor turn
  double xInches = 0.0;
  double yInches = 0.0;
};

// Distance sensors always carry a full pose and their calibrated gain/offset (SPEC 3, 6).
struct DistanceSensorConfig {
  DistanceSensor* sensor = nullptr;
  double xInches = 0.0;
  double yInches = 0.0;
  double headingDegrees = 0.0;  // which way it points, relative to the robot's front
  double gain = 1.0;
  double offsetInches = 0.0;
};

// The GPS sensor, where it sits on the robot, and how its field frame maps onto the team's
// (the team's frame = the GPS frame turned clockwise by fieldRotationDegrees, then shifted).
// Left out until its mount is known (SPEC 20.14, 21.3).
struct GpsConfig {
  GpsSensor* sensor = nullptr;
  double xInches = 0.0;         // right of the robot's center
  double yInches = 0.0;         // forward of it
  double headingDegrees = 0.0;  // the way it faces; 180 for the usual rear mount
  double fieldXOffsetInches = 0.0;
  double fieldYOffsetInches = 0.0;
  double fieldRotationDegrees = 0.0;
};

// Orientation is required; position is optional metadata (heading doesn't depend on it).
struct ImuConfig {
  Imu* sensor = nullptr;
  ImuOrientation orientation = ImuOrientation::Flat;
  double xInches = 0.0;
  double yInches = 0.0;
};

// Rated output RPM of a cartridge.
constexpr double cartridgeRpm(Cartridge cartridge) {
  switch (cartridge) {
    case Cartridge::Red100:
      return 100.0;
    case Cartridge::Green200:
      return 200.0;
    case Cartridge::Blue600:
      return 600.0;
  }
  return 0.0;
}

// The 5.5 W motor has no swappable cartridge: its output is always 200 rpm.
constexpr double EXP_5_5W_RPM = 200.0;

// Output RPM of a motor: the cartridge's for an 11 W motor, 200 for a 5.5 W one.
constexpr double motorRpm(MotorType type, Cartridge cartridge) {
  return type == MotorType::Exp5_5W ? EXP_5_5W_RPM : cartridgeRpm(cartridge);
}
constexpr double motorRpm(const MotorConfig& motor) {
  return motorRpm(motor.type, motor.cartridge);
}

// The gearset PROS should be told: a 5.5 W motor reports as green.
constexpr Cartridge effectiveCartridge(const MotorConfig& motor) {
  return motor.type == MotorType::Exp5_5W ? Cartridge::Green200 : motor.cartridge;
}

// External gearing between a motor and its wheel: {motorTeeth, wheelTeeth}, e.g. {36, 48} turns
// 600 rpm into 450, or a ratio (wheel turns per motor turn). No default: a drive can't work
// without it (SPEC 20.14).
class Gearing {
 public:
  constexpr Gearing(double motorTeeth, double wheelTeeth) : m_ratio(motorTeeth / wheelTeeth) {}
  constexpr Gearing(double ratio) : m_ratio(ratio) {}  // a plain ratio

  // G_ext: wheel turns per motor output turn.
  constexpr double ratio() const { return m_ratio; }

 private:
  double m_ratio;
};

// wheelRpm = motorRpm * motorTeeth / wheelTeeth (SPEC 20.1).
constexpr double wheelRpm(double motorOutputRpm, const Gearing& gearing) {
  return motorOutputRpm * gearing.ratio();
}

// Approximate measured rolling diameters of VEX omni wheels, in inches: the values LemLib ships
// (credited in CREDITS.md). Replace with your own measured value after the push test (SPEC 20.2).
// A plain number is always allowed instead.
namespace Omniwheel {
constexpr double NEW_2 = 2.125;
constexpr double NEW_275 = 2.75;
constexpr double OLD_275 = 2.75;
constexpr double NEW_275_HALF = 2.744;
constexpr double OLD_275_HALF = 2.74;
constexpr double NEW_325 = 3.25;
constexpr double OLD_325 = 3.25;
constexpr double NEW_325_HALF = 3.246;
constexpr double OLD_325_HALF = 3.246;
constexpr double NEW_4 = 4.0;
constexpr double OLD_4 = 4.18;
constexpr double NEW_4_HALF = 3.995;
constexpr double OLD_4_HALF = 4.175;
}  // namespace Omniwheel

}  // namespace cardLib
