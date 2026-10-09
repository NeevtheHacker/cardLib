// terminal_log.hpp
// Why: turns the telemetry ring into CSV on the terminal, off the control loops (SPEC 11).
// Owns: the terminal task (priority 2, every 100 ms) and its formatting.
#pragma once

#include <cstddef>
#include <cstdint>

#include "cardLib/chassis/telemetry_buffer.hpp"
#include "pros/rtos.hpp"

namespace cardLib {

class TerminalLog {
 public:
  static constexpr std::uint32_t PRIORITY = 2;
  static constexpr std::uint32_t PERIOD_MS = 100;
  static constexpr std::size_t MAX_LINES_PER_FLUSH = 20;  // so it never floods the link

  explicit TerminalLog(TelemetryBuffer& telemetry) : m_telemetry(telemetry) {}

  // Starts the task. With no telemetry channels on (a match build) it prints nothing.
  void start();

 private:
  static void taskEntry(void* self);
  void run();
  void flush();

  TelemetryBuffer& m_telemetry;
  bool m_headerPrinted = false;
  std::uint32_t m_droppedReported = 0;
  pros::task_t m_task = nullptr;
};

}  // namespace cardLib
