// fake_devices.cpp
// Why: laptop versions of the hardware/ wrappers (MotorGroup, Imu, RotationSensor,
//      DistanceSensor, GpsSensor) that read and write the fake state instead of a V5 port.
// Owns: the fake device state, one entry per port, behind one mutex. Test-only.
#include "fakes/fake_devices.hpp"

#include <array>
#include <mutex>

#include "cardLib/hardware/distance_sensor.hpp"
#include "cardLib/hardware/imu.hpp"
#include "cardLib/hardware/motor_group.hpp"
#include "cardLib/hardware/port_registry.hpp"
#include "cardLib/hardware/rotation_sensor.hpp"

namespace fake {
namespace {

constexpr std::size_t PORTS = 22;  // 1 to 21; 0 unused
constexpr int DEVICE_MOTOR = 2;
constexpr int DEVICE_ROTATION = 4;
constexpr int DEVICE_IMU = 6;
constexpr int DEVICE_DISTANCE = 7;
constexpr int DEVICE_GPS = 20;

struct PortState {
  int pluggedType = 0;
  std::int32_t millivolts = 0;
  double degrees = 0.0;
  bool wiredBackwards = false;
  std::optional<double> distanceInches;
  std::optional<cardLib::GpsSample> gps;
};

struct State {
  std::array<PortState, PORTS> ports = {};
  double imuRotationDegrees = 0.0;
  bool imuPresent = true;
};

std::mutex g_mutex;
State g_state;

PortState& port(std::uint8_t index) { return g_state.ports[index < PORTS ? index : 0]; }

}  // namespace

void reset() {
  const std::lock_guard<std::mutex> lock(g_mutex);
  g_state = {};
}

void plug(std::uint8_t index, int deviceType) {
  const std::lock_guard<std::mutex> lock(g_mutex);
  port(index).pluggedType = deviceType;
}

int pluggedType(std::uint8_t index) {
  const std::lock_guard<std::mutex> lock(g_mutex);
  return port(index).pluggedType;
}

void plugMotor(std::uint8_t index) { plug(index, DEVICE_MOTOR); }
void plugImu(std::uint8_t index) { plug(index, DEVICE_IMU); }
void plugRotation(std::uint8_t index) { plug(index, DEVICE_ROTATION); }
void plugDistance(std::uint8_t index) { plug(index, DEVICE_DISTANCE); }
void plugGps(std::uint8_t index) { plug(index, DEVICE_GPS); }

double batteryPercent() { return 100.0; }

std::int32_t motorMillivolts(std::uint8_t index) {
  const std::lock_guard<std::mutex> lock(g_mutex);
  return port(index).millivolts;
}

void setMotorDegrees(std::uint8_t index, double degrees) {
  const std::lock_guard<std::mutex> lock(g_mutex);
  port(index).degrees = degrees;
}

void setMotorWiredBackwards(std::uint8_t index, bool backwards) {
  const std::lock_guard<std::mutex> lock(g_mutex);
  port(index).wiredBackwards = backwards;
}

void setImuRotation(double degrees) {
  const std::lock_guard<std::mutex> lock(g_mutex);
  g_state.imuRotationDegrees = degrees;
}

void setImuPresent(bool present) {
  const std::lock_guard<std::mutex> lock(g_mutex);
  g_state.imuPresent = present;
}

void setDistanceInches(std::uint8_t index, std::optional<double> inches) {
  const std::lock_guard<std::mutex> lock(g_mutex);
  port(index).distanceInches = inches;
}

void setGpsSample(std::uint8_t index, std::optional<cardLib::GpsSample> sample) {
  const std::lock_guard<std::mutex> lock(g_mutex);
  port(index).gps = sample;
}

}  // namespace fake

namespace cardLib {

MotorGroup::MotorGroup(std::initializer_list<MotorConfig> motors, const char* role)
    : m_role(role) {
  for (const MotorConfig& motor : motors) {
    if (m_count == MAX_MOTORS_PER_GROUP) {
      m_tooManyMotors = true;
      break;
    }
    m_motors[m_count++] = motor;
  }
}

bool MotorGroup::initialize() { return !m_tooManyMotors; }

std::optional<double> MotorGroup::readDegrees() {
  if (m_count == 0) {
    return std::nullopt;
  }
  const std::lock_guard<std::mutex> lock(fake::g_mutex);
  double sum = 0.0;
  for (std::size_t i = 0; i < m_count; ++i) {
    sum += fake::port(m_motors[i].port).degrees;
  }
  m_answeringCount = m_count;
  return sum / static_cast<double>(m_count);
}

bool MotorGroup::reset() {
  const std::lock_guard<std::mutex> lock(fake::g_mutex);
  for (std::size_t i = 0; i < m_count; ++i) {
    fake::port(m_motors[i].port).degrees = 0.0;
  }
  return true;
}

void MotorGroup::registerPorts(PortRegistry& registry, const char* role,
                               Requirement requirement) const {
  for (std::size_t i = 0; i < m_count; ++i) {
    registry.add(m_motors[i].port, DeviceKind::Motor, role, requirement);
  }
}

void MotorGroup::moveMillivolts(std::int32_t millivolts) {
  const std::lock_guard<std::mutex> lock(fake::g_mutex);
  for (std::size_t i = 0; i < m_count; ++i) {
    fake::port(m_motors[i].port).millivolts = millivolts;
  }
}

void MotorGroup::stop(BrakeMode) { moveMillivolts(0); }

std::optional<double> MotorGroup::currentAmps(std::size_t) const { return 0.0; }

std::optional<double> MotorGroup::temperatureCelsius(std::size_t) const { return 30.0; }

std::optional<double> MotorGroup::velocityRpm(std::size_t index) const {
  const std::lock_guard<std::mutex> lock(fake::g_mutex);
  const fake::PortState& state = fake::port(m_motors[index].port);
  const double rpm = motorRpm(m_motors[index]) * state.millivolts / 12000.0;
  return state.wiredBackwards ? -rpm : rpm;
}

bool Imu::startCalibration() {
  const std::lock_guard<std::mutex> lock(fake::g_mutex);
  return fake::g_state.imuPresent;
}

bool Imu::isCalibrating() const { return false; }

std::optional<double> Imu::readRotationDegrees() {
  const std::lock_guard<std::mutex> lock(fake::g_mutex);
  if (!fake::g_state.imuPresent) {
    return std::nullopt;
  }
  return fake::g_state.imuRotationDegrees;
}

std::optional<double> Imu::readGyroRateDegreesPerSecond() { return 0.0; }

std::optional<ImuOrientation> Imu::physicalOrientation() const { return ImuOrientation::Flat; }

bool RotationSensor::initialize() { return true; }

std::optional<double> RotationSensor::readDegrees() {
  const std::lock_guard<std::mutex> lock(fake::g_mutex);
  return fake::port(m_port).degrees;
}

bool RotationSensor::reset() {
  const std::lock_guard<std::mutex> lock(fake::g_mutex);
  fake::port(m_port).degrees = 0.0;
  return true;
}

void RotationSensor::registerPorts(PortRegistry& registry, const char* role,
                                   Requirement requirement) const {
  registry.add(m_port, DeviceKind::Rotation, role, requirement);
}

std::optional<double> DistanceSensor::readInches() {
  const std::lock_guard<std::mutex> lock(fake::g_mutex);
  return fake::port(m_port).distanceInches;
}

std::optional<double> DistanceSensor::readConfidence() { return 1.0; }

std::optional<GpsSample> GpsSensor::read() {
  const std::lock_guard<std::mutex> lock(fake::g_mutex);
  return fake::port(m_port).gps;
}

}  // namespace cardLib
