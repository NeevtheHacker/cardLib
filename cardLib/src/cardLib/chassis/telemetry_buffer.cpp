// telemetry_buffer.cpp
// Why: implements the fixed telemetry ring.
// Owns: nothing beyond TelemetryBuffer's own state.
#include "cardLib/chassis/telemetry_buffer.hpp"

namespace cardLib {

void TelemetryBuffer::push(const TelemetryRecord& record) {
  m_mutex.take();
  const std::size_t slot = (m_oldest + m_count) % CAPACITY;
  m_records[slot] = record;
  if (m_count == CAPACITY) {
    m_oldest = (m_oldest + 1) % CAPACITY;
    ++m_dropped;
  } else {
    ++m_count;
  }
  m_mutex.give();
}

std::size_t TelemetryBuffer::drain(std::span<TelemetryRecord> out) {
  m_mutex.take();
  std::size_t moved = 0;
  while (moved < out.size() && m_count > 0) {
    out[moved++] = m_records[m_oldest];
    m_oldest = (m_oldest + 1) % CAPACITY;
    --m_count;
  }
  m_mutex.give();
  return moved;
}

std::uint32_t TelemetryBuffer::droppedCount() {
  m_mutex.take();
  const std::uint32_t dropped = m_dropped;
  m_mutex.give();
  return dropped;
}

}  // namespace cardLib
