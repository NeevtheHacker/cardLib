// sensor_status.hpp
// Why: one OK / Suspect / Faulted / Missing word per configured sensor, for the Sensors page
//      (SPEC 21.4), decided here so the screen code holds no robot logic.
// Owns: SensorStatus and sensorStatus(). No state.
#pragma once

#include <array>
#include <cstddef>

#include "cardLib/chassis/chassis.hpp"
#include "cardLib/chassis/chassis_config.hpp"
#include "cardLib/chassis/odometry_task.hpp"

namespace cardLib {

// Ok: working. Suspect: answering, but this loop's reading is doubtful or absent. Faulted:
// stopped answering mid-run (three bad reads in a row). Missing: not found at startup.
enum class SensorStatus { Ok, Suspect, Faulted, Missing };

struct SensorStatusLine {
  const char* name = "";
  SensorStatus status = SensorStatus::Ok;
};

constexpr std::size_t MAX_SENSOR_STATUS_LINES = 16;

struct SensorStatusList {
  std::array<SensorStatusLine, MAX_SENSOR_STATUS_LINES> lines = {};
  std::size_t count = 0;
};

// Every configured sensor, in a fixed order: IMU, tracking wheels, drive, distance sensors,
// GPS. Sensors the robot doesn't have are left out.
SensorStatusList sensorStatus(const ChassisConfig& config, const StartupReport& startup,
                              const OdometrySnapshot& odometry);

const char* sensorStatusName(SensorStatus status);

}  // namespace cardLib
