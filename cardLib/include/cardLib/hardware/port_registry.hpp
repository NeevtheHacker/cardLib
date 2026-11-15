// port_registry.hpp
// Why: catches port mistakes twice: duplicates and impossible ports at compile time, missing or
//      wrong devices at startup (SPEC sections 3 and 17).
// Owns: the list of registered ports and the checks over them.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "cardLib/hardware/hardware_config.hpp"

namespace cardLib {

constexpr std::uint8_t MIN_SMART_PORT = 1;
constexpr std::uint8_t MAX_SMART_PORT = 21;

// 3-wire ports may be written 1 to 8 or 'A' to 'H' / 'a' to 'h'. Returns 0 if invalid.
constexpr std::uint8_t normalizeAdiPort(std::uint8_t port) {
  if (port >= 1 && port <= 8) {
    return port;
  }
  if (port >= 'A' && port <= 'H') {
    return static_cast<std::uint8_t>(port - 'A' + 1);
  }
  if (port >= 'a' && port <= 'h') {
    return static_cast<std::uint8_t>(port - 'a' + 1);
  }
  return 0;
}

constexpr bool isValidSmartPort(std::uint8_t port) {
  return port >= MIN_SMART_PORT && port <= MAX_SMART_PORT;
}

// For static_assert in ports.hpp: every Smart Port in 1 to 21, none used twice.
constexpr bool smartPortsValid(std::span<const std::uint8_t> ports) {
  for (std::size_t i = 0; i < ports.size(); ++i) {
    if (!isValidSmartPort(ports[i])) {
      return false;
    }
    for (std::size_t j = i + 1; j < ports.size(); ++j) {
      if (ports[i] == ports[j]) {
        return false;
      }
    }
  }
  return true;
}

// For static_assert in ports.hpp: every 3-wire port valid, none used twice.
constexpr bool adiPortsValid(std::span<const std::uint8_t> ports) {
  for (std::size_t i = 0; i < ports.size(); ++i) {
    if (normalizeAdiPort(ports[i]) == 0) {
      return false;
    }
    for (std::size_t j = i + 1; j < ports.size(); ++j) {
      if (normalizeAdiPort(ports[i]) == normalizeAdiPort(ports[j])) {
        return false;
      }
    }
  }
  return true;
}

// Smart Port kinds are compared with what PROS sees plugged in. 3-wire ports can't identify
// their device, so for those only the configured mode is checked.
// AdiEncoderBottom is the second port of a 3-wire encoder: only checked for duplicates.
enum class DeviceKind {
  Motor,
  Imu,
  Rotation,
  Distance,
  Gps,
  AdiEncoder,
  AdiEncoderBottom,
  AdiDigitalOut,
};

enum class PortProblem { InvalidPort, DuplicatePort, Missing, WrongDevice, WrongAdiMode };

struct PortIssue {
  std::uint8_t port = 0;
  DeviceKind kind = DeviceKind::Motor;
  const char* role = "";
  Requirement requirement = Requirement::Required;
  PortProblem problem = PortProblem::Missing;
};

constexpr std::size_t MAX_REGISTERED_PORTS = 32;

class PortRegistry {
 public:
  // False if the registry is full.
  bool add(std::uint8_t port, DeviceKind kind, const char* role,
           Requirement requirement = Requirement::Required);

  // Runs every check and writes up to issues.size() problems. Returns how many were found.
  std::size_t validate(std::span<PortIssue> issues) const;

 private:
  struct Entry {
    std::uint8_t port = 0;
    DeviceKind kind = DeviceKind::Motor;
    const char* role = "";
    Requirement requirement = Requirement::Required;
  };

  bool sharesPort(std::size_t index) const;

  std::array<Entry, MAX_REGISTERED_PORTS> m_entries = {};
  std::size_t m_count = 0;
};

constexpr bool isAdiKind(DeviceKind kind) {
  return kind == DeviceKind::AdiEncoder || kind == DeviceKind::AdiEncoderBottom ||
         kind == DeviceKind::AdiDigitalOut;
}

}  // namespace cardLib
