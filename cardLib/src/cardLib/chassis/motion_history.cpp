// motion_history.cpp
// Why: implements the fixed motion-history ring.
// Owns: nothing beyond MotionHistory's own state.
#include "cardLib/chassis/motion_history.hpp"

namespace cardLib {

void MotionHistory::add(const MotionRecord& record) {
  m_mutex.take();
  m_records[(m_oldest + m_count) % CAPACITY] = record;
  if (m_count == CAPACITY) {
    m_oldest = (m_oldest + 1) % CAPACITY;
  } else {
    ++m_count;
  }
  m_mutex.give();
}

std::size_t MotionHistory::copyAll(std::span<MotionRecord> out) {
  m_mutex.take();
  std::size_t copied = 0;
  while (copied < out.size() && copied < m_count) {
    out[copied] = m_records[(m_oldest + copied) % CAPACITY];
    ++copied;
  }
  m_mutex.give();
  return copied;
}

}  // namespace cardLib
