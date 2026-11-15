// black_box.cpp
// Why: implements the black box dump as CSV.
// Owns: nothing beyond BlackBox's own state.
#include "cardLib/systems/black_box.hpp"

#include <array>
#include <cstdio>

#include "pros/misc.h"

namespace cardLib {

BlackBox::SaveResult BlackBox::saveToSd(const char* path) {
  if (m_chassis.isMoving()) {
    m_lastResult = SaveResult::Busy;
    return m_lastResult;
  }
  if (pros::c::usd_is_installed() != 1) {
    m_lastResult = SaveResult::NoCard;
    return m_lastResult;
  }
  std::FILE* file = std::fopen(path, "w");
  if (file == nullptr) {
    m_lastResult = SaveResult::OpenFailed;
    return m_lastResult;
  }

  std::array<MotionRecord, MotionHistory::CAPACITY> records = {};
  const std::size_t count = m_chassis.motionHistory().copyAll(records);
  std::fprintf(file,
               "start_ms,kind,target_x,target_y,target_h,start_x,start_y,start_h,"
               "end_x,end_y,end_h,result,duration_ms\n");
  for (std::size_t i = 0; i < count; ++i) {
    const MotionRecord& r = records[i];
    std::fprintf(file, "%lu,%d,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f,%d,%lu\n",
                 static_cast<unsigned long>(r.startTimeMs), static_cast<int>(r.kind),
                 r.target.xInches, r.target.yInches, r.target.headingDegrees, r.startPose.xInches,
                 r.startPose.yInches, r.startPose.headingDegrees, r.endPose.xInches,
                 r.endPose.yInches, r.endPose.headingDegrees, static_cast<int>(r.result),
                 static_cast<unsigned long>(r.durationMs));
  }
  std::fclose(file);
  m_lastResult = SaveResult::Saved;
  return m_lastResult;
}

}  // namespace cardLib
