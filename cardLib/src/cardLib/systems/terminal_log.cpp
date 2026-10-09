// terminal_log.cpp
// Why: implements the terminal task: drain, format as CSV, print, with a line cap.
// Owns: nothing beyond TerminalLog's own state. The only cardLib code that prints telemetry.
#include "cardLib/systems/terminal_log.hpp"

#include <array>
#include <cstdio>
#include <span>

namespace cardLib {

void TerminalLog::start() {
  if (m_task != nullptr) {
    return;
  }
  m_task = pros::c::task_create(&TerminalLog::taskEntry, this, PRIORITY,
                                TASK_STACK_DEPTH_DEFAULT, "cardLib terminal");
}

void TerminalLog::taskEntry(void* self) { static_cast<TerminalLog*>(self)->run(); }

void TerminalLog::run() {
  std::uint32_t wakeTimeMs = pros::millis();
  while (true) {
    flush();
    pros::Task::delay_until(&wakeTimeMs, PERIOD_MS);
  }
}

void TerminalLog::flush() {
  std::array<TelemetryRecord, MAX_LINES_PER_FLUSH> batch = {};
  const std::size_t count = m_telemetry.drain(batch);
  if (count > 0 && !m_headerPrinted) {
    std::printf("t,ch,target,meas,out\n");
    m_headerPrinted = true;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const TelemetryRecord& record = batch[i];
    std::printf("%lu,%u,%.3f,%.3f,%.3f\n", static_cast<unsigned long>(record.timeMs),
                static_cast<unsigned>(record.channel), record.target, record.measured,
                record.output);
  }
  const std::uint32_t dropped = m_telemetry.droppedCount();
  if (dropped != m_droppedReported) {
    std::printf("# dropped %lu records\n", static_cast<unsigned long>(dropped - m_droppedReported));
    m_droppedReported = dropped;
  }
}

}  // namespace cardLib
