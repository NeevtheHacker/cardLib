// hardware_config.hpp
// Why: the robot's physical description, one row per device (SPEC sections 3 and 17).
// Owns: plain config structs and enums. No behavior.
#pragma once

#include <cstdint>

namespace cardLib {

class Encoder;
class Imu;
class DistanceSensor;

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

}  // namespace cardLib
