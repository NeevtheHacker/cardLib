// sensor_health.hpp
// Why: remembers whether a device has been answering, so diagnostics can say what dropped out.
// Owns: one device's read counters. It never decides fallbacks; odometry does (SPEC section 4).
#pragma once

#include <cstdint>

namespace cardLib {

class SensorHealth {
 public:
  // One failed read is a glitch; this many in a row is a fault (SPEC section 4).
  static constexpr std::uint32_t FAILURES_TO_FAULT = 3;

  // Call once per read with whether PROS returned a value.
  void record(bool readOk) {
    if (readOk) {
      m_everAnswered = true;
      m_failuresInARow = 0;
      return;
    }
    ++m_failuresInARow;
    ++m_totalFailures;
  }

  bool answering() const { return m_everAnswered && m_failuresInARow == 0; }
  bool everAnswered() const { return m_everAnswered; }
  bool faulted() const { return m_failuresInARow >= FAILURES_TO_FAULT; }
  std::uint32_t failuresInARow() const { return m_failuresInARow; }
  std::uint32_t totalFailures() const { return m_totalFailures; }

 private:
  bool m_everAnswered = false;
  std::uint32_t m_failuresInARow = 0;
  std::uint32_t m_totalFailures = 0;
};

}  // namespace cardLib
