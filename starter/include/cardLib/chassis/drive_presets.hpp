// drive_presets.hpp
// Why: one-line drive setups (SPEC 20.4): a tank, an asterisk, or any list of wheel modules,
//      plus the speed-match report that can stop the build (SPEC 20.7).
// Owns: WheelModule, AsteriskConfig and the preset functions. All constexpr, so a drive built
//       at namespace scope can be checked with static_assert before the code is compiled.
#pragma once

#include <cstddef>
#include <initializer_list>
#include <numbers>

#include "cardLib/algorithms/control/drive_kinematics.hpp"
#include "cardLib/chassis/chassis_config.hpp"
#include "cardLib/hardware/hardware_config.hpp"
#include "cardLib/hardware/motor_group.hpp"
#include "cardLib/hardware/required_value.hpp"

namespace cardLib {

// One driven wheel (SPEC 20.1). The diameter, gearing and roll angle have no default, so
// leaving one out doesn't compile (SPEC 20.14). Positions are the wheel's contact point, x
// right and y forward of the robot's center. Set reverse flags so a positive command pushes
// the robot forward; the roll angle says which way forward leans (SPEC 20.3).
struct WheelModule {
  MotorGroup* motors = nullptr;
  RequiredValue<double> wheelDiameterInches;  // Omniwheel:: or your measured value
  Gearing gearing;                            // {motorTeeth, wheelTeeth} or a ratio
  double xInches = 0.0;
  double yInches = 0.0;
  RequiredValue<double> rollAngleDegrees;  // clockwise from forward
};

constexpr DriveModule toDriveModule(const WheelModule& module) {
  return {module.motors,          module.wheelDiameterInches.value(),
          module.gearing.ratio(), module.xInches,
          module.yInches,         module.rollAngleDegrees.value()};
}

// Any drive: list the modules in order (for X and asterisk drives: FrontLeft, FrontRight,
// BackLeft, BackRight, then CenterLeft, CenterRight). Modules past MAX_WHEEL_MODULES are ignored.
constexpr DriveConfig customDrive(std::initializer_list<WheelModule> modules) {
  DriveConfig drive;
  for (const WheelModule& module : modules) {
    if (drive.moduleCount == MAX_WHEEL_MODULES) {
      break;
    }
    drive.modules[drive.moduleCount++] = toDriveModule(module);
  }
  return drive;
}

// Tank: one module per side at x = -W/2 and +W/2, rolling forward.
constexpr DriveConfig tank(MotorGroup& left, MotorGroup& right, double wheelDiameterInches,
                           Gearing gearing, double trackWidthInches) {
  const double halfTrack = trackWidthInches / 2.0;
  DriveConfig drive = customDrive({
      {.motors = &left,
       .wheelDiameterInches = wheelDiameterInches,
       .gearing = gearing,
       .xInches = -halfTrack,
       .rollAngleDegrees = 0.0},
      {.motors = &right,
       .wheelDiameterInches = wheelDiameterInches,
       .gearing = gearing,
       .xInches = halfTrack,
       .rollAngleDegrees = 0.0},
  });
  drive.left = &left;
  drive.right = &right;
  return drive;
}

// Asterisk: four 45 degree corners plus two forward-rolling center wheels. Every number that
// describes the build is required; they are the team's own measurements (SPEC 20.14).
struct AsteriskConfig {
  std::array<MotorGroup*, 4> corners = {};  // FrontLeft, FrontRight, BackLeft, BackRight
  RequiredValue<double> cornerWheel;
  Gearing cornerGearing;
  RequiredValue<double> cornerHalfWidthInches;   // corner contact point, right of center
  RequiredValue<double> cornerHalfLengthInches;  // and forward of center
  std::array<MotorGroup*, 2> centers = {};       // CenterLeft, CenterRight
  RequiredValue<double> centerWheel;
  Gearing centerGearing;
  RequiredValue<double> centerHalfWidthInches;
};

// FrontLeft and BackRight roll at +45 degrees, FrontRight and BackLeft at -45 (SPEC 20.3).
constexpr DriveConfig asterisk(const AsteriskConfig& config) {
  const double w = config.cornerHalfWidthInches;
  const double l = config.cornerHalfLengthInches;
  const double c = config.centerHalfWidthInches;
  const auto corner = [&config](MotorGroup* motors, double x, double y, double roll) {
    return WheelModule{.motors = motors,
                       .wheelDiameterInches = config.cornerWheel,
                       .gearing = config.cornerGearing,
                       .xInches = x,
                       .yInches = y,
                       .rollAngleDegrees = roll};
  };
  const auto center = [&config](MotorGroup* motors, double x) {
    return WheelModule{.motors = motors,
                       .wheelDiameterInches = config.centerWheel,
                       .gearing = config.centerGearing,
                       .xInches = x,
                       .rollAngleDegrees = 0.0};
  };
  return customDrive({
      corner(config.corners[0], -w, l, 45.0),
      corner(config.corners[1], w, l, -45.0),
      corner(config.corners[2], -w, -l, -45.0),
      corner(config.corners[3], w, -l, 45.0),
      center(config.centers[0], -c),
      center(config.centers[1], c),
  });
}

// The kinematics of a drive built from modules, with every motor of this type and cartridge.
// constexpr: the motor groups themselves are never read.
constexpr DriveKinematics presetKinematics(const DriveConfig& drive, Cartridge cartridge,
                                           MotorType type = MotorType::V5_11W) {
  std::array<WheelSpec, MAX_WHEEL_MODULES> wheels = {};
  for (std::size_t i = 0; i < drive.moduleCount; ++i) {
    const DriveModule& module = drive.modules[i];
    const double rpm = motorRpm(type, cartridge) * module.gearRatio;
    wheels[i] = {module.xInches, module.yInches, module.rollAngleDegrees,
                 std::numbers::pi * module.wheelDiameterInches * rpm / 60.0};
  }
  return DriveKinematics(std::span<const WheelSpec>(wheels.data(), drive.moduleCount));
}

// The SPEC 20.6 numbers before the code is compiled. In robot code:
//   static_assert(cardLib::requireSpeedMatchWithin(
//       cardLib::speedMatch(DRIVE, cardLib::Cartridge::Blue600), 0.10));
constexpr SpeedMatchReport speedMatch(const DriveConfig& drive, Cartridge cartridge,
                                      MotorType type = MotorType::V5_11W) {
  return speedMatchReport(presetKinematics(drive, cartridge, type));
}

}  // namespace cardLib
