// port_registry.cpp
// Why: implements the startup port checks against what PROS sees plugged in.
// Owns: nothing beyond PortRegistry's own list.
#include "cardLib/hardware/port_registry.hpp"

#include <optional>

#include "pros/adi.h"
#include "pros/device.h"

namespace cardLib {

namespace {

pros::c::v5_device_e_t expectedSmartDevice(DeviceKind kind) {
  switch (kind) {
    case DeviceKind::Motor:
      return pros::c::E_DEVICE_MOTOR;
    case DeviceKind::Imu:
      return pros::c::E_DEVICE_IMU;
    case DeviceKind::Rotation:
      return pros::c::E_DEVICE_ROTATION;
    case DeviceKind::Distance:
      return pros::c::E_DEVICE_DISTANCE;
    case DeviceKind::AdiEncoder:
    case DeviceKind::AdiEncoderBottom:
    case DeviceKind::AdiDigitalOut:
      break;
  }
  return pros::c::E_DEVICE_UNDEFINED;
}

pros::adi_port_config_e_t expectedAdiMode(DeviceKind kind) {
  return kind == DeviceKind::AdiEncoder ? pros::E_ADI_LEGACY_ENCODER : pros::E_ADI_DIGITAL_OUT;
}

// Empty if the device on this port looks right.
std::optional<PortProblem> checkPluggedDevice(std::uint8_t port, DeviceKind kind) {
  if (isAdiKind(kind)) {
    const std::uint8_t adiPort = normalizeAdiPort(port);
    if (adiPort == 0) {
      return PortProblem::InvalidPort;
    }
    if (kind == DeviceKind::AdiEncoderBottom) {
      return std::nullopt;
    }
    if (pros::c::adi_port_get_config(adiPort) != expectedAdiMode(kind)) {
      return PortProblem::WrongAdiMode;
    }
    return std::nullopt;
  }
  if (!isValidSmartPort(port)) {
    return PortProblem::InvalidPort;
  }
  const pros::c::v5_device_e_t plugged = pros::c::get_plugged_type(port);
  if (plugged == pros::c::E_DEVICE_NONE) {
    return PortProblem::Missing;
  }
  if (plugged != expectedSmartDevice(kind)) {
    return PortProblem::WrongDevice;
  }
  return std::nullopt;
}

}  // namespace

bool PortRegistry::add(std::uint8_t port, DeviceKind kind, const char* role,
                       Requirement requirement) {
  if (m_count == MAX_REGISTERED_PORTS) {
    return false;
  }
  m_entries[m_count++] = {port, kind, role, requirement};
  return true;
}

// 3-wire ports and Smart Ports are separate numbering, so only compare within one kind.
bool PortRegistry::sharesPort(std::size_t index) const {
  const Entry& entry = m_entries[index];
  for (std::size_t other = 0; other < index; ++other) {
    const Entry& earlier = m_entries[other];
    if (isAdiKind(entry.kind) != isAdiKind(earlier.kind)) {
      continue;
    }
    const bool same = isAdiKind(entry.kind)
                          ? normalizeAdiPort(entry.port) == normalizeAdiPort(earlier.port)
                          : entry.port == earlier.port;
    if (same) {
      return true;
    }
  }
  return false;
}

std::size_t PortRegistry::validate(std::span<PortIssue> issues) const {
  std::size_t found = 0;
  for (std::size_t i = 0; i < m_count; ++i) {
    const Entry& entry = m_entries[i];
    std::optional<PortProblem> problem = checkPluggedDevice(entry.port, entry.kind);
    if (sharesPort(i)) {
      problem = PortProblem::DuplicatePort;
    }
    if (!problem.has_value()) {
      continue;
    }
    if (found < issues.size()) {
      issues[found] = {entry.port, entry.kind, entry.role, entry.requirement, *problem};
    }
    ++found;
  }
  return found;
}

}  // namespace cardLib
