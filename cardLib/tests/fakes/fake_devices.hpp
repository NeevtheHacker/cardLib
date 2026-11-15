// fake_devices.hpp
// Why: the test side of the fake hardware: tests set what each port's device reads and check
//      what the chassis commanded, without a robot.
// Owns: the fake device state API. Test-only; never part of the robot build.
#pragma once

#include <cstdint>
#include <optional>

#include "cardLib/hardware/gps_sensor.hpp"
#include "cardLib/hardware/hardware_config.hpp"

namespace fake {

// Clears every device, unplugs every port and sets the battery to 100 %. Call first in a test.
void reset();

// Stops every fake task thread and waits for it. Call before the chassis that started them is
// destroyed (TaskGuard does it).
void stopTasks();

// Declare after the chassis, so it is destroyed (and stops the tasks) first.
struct TaskGuard {
  TaskGuard() = default;
  TaskGuard(const TaskGuard&) = delete;
  TaskGuard& operator=(const TaskGuard&) = delete;
  ~TaskGuard() { stopTasks(); }
};

// What get_plugged_type() reports; 0 (E_DEVICE_NONE) until plugged.
void plug(std::uint8_t port, int deviceType);
int pluggedType(std::uint8_t port);
void plugMotor(std::uint8_t port);
void plugImu(std::uint8_t port);
void plugRotation(std::uint8_t port);
void plugDistance(std::uint8_t port);
void plugGps(std::uint8_t port);

double batteryPercent();

// Motors are in the robot's convention: a positive command and a positive reading are the
// motor turning the way that pushes the robot as its module's roll angle says (reverse flags
// already applied). velocityRpm is a perfect motor: command / 12 V of its rated rpm, unless the
// port is wired backwards.
std::int32_t motorMillivolts(std::uint8_t port);
void setMotorDegrees(std::uint8_t port, double degrees);
void setMotorWiredBackwards(std::uint8_t port, bool backwards);

void setImuRotation(double degrees);
void setImuPresent(bool present);

void setDistanceInches(std::uint8_t port, std::optional<double> inches);
void setGpsSample(std::uint8_t port, std::optional<cardLib::GpsSample> sample);

}  // namespace fake
