// motion_history.hpp
// Why: the black box's memory: the last 64 motions, kept in RAM (SPEC section 11).
// Owns: a fixed ring of motion records. Writer: motion task. Reader: the black box (systems).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "cardLib/algorithms/math/pose.hpp"
#include "cardLib/algorithms/motion/motion_result.hpp"
#include "cardLib/chassis/motion_command.hpp"
#include "pros/rtos.hpp"

namespace cardLib {

struct MotionRecord {
  MotionKind kind = MotionKind::None;
  Pose target = {};
  Pose startPose = {};
  Pose endPose = {};
  MotionResult result = MotionResult::Settled;
  std::uint32_t startTimeMs = 0;
  std::uint32_t durationMs = 0;
};

class MotionHistory {
 public:
  static constexpr std::size_t CAPACITY = 64;

  void add(const MotionRecord& record);

  // Copies up to out.size() records, oldest first. Returns how many.
  std::size_t copyAll(std::span<MotionRecord> out);

 private:
  pros::Mutex m_mutex;
  std::array<MotionRecord, CAPACITY> m_records = {};
  std::size_t m_oldest = 0;
  std::size_t m_count = 0;
};

}  // namespace cardLib
