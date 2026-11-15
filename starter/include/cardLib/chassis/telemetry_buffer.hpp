// telemetry_buffer.hpp
// Why: control loops log fixed-size records with no text formatting; the terminal task formats
//      them later (SPEC section 11).
// Owns: a fixed ring of records. Producers: odometry and motion tasks. Consumer: terminal task.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "pros/rtos.hpp"

namespace cardLib {

// WallReset: target = corrected coordinate, measured = previous, output = axis (0 = x, 1 = y).
// GpsReset: target = change in x, measured = change in y, output = change in heading.
// Resets are rare and always printed, as one readable line each (SPEC 21.4).
enum class TelemetryChannel : std::uint8_t { Motion = 1, Pose = 2, WallReset = 3, GpsReset = 4 };

struct TelemetryRecord {
  std::uint32_t timeMs = 0;
  TelemetryChannel channel = TelemetryChannel::Motion;
  double target = 0.0;
  double measured = 0.0;
  double output = 0.0;
};

class TelemetryBuffer {
 public:
  static constexpr std::size_t CAPACITY = 256;

  // Adds a record; when full, the oldest is overwritten and counted as dropped.
  void push(const TelemetryRecord& record);

  // Moves up to out.size() of the oldest records into out. Returns how many.
  std::size_t drain(std::span<TelemetryRecord> out);

  std::uint32_t droppedCount();

 private:
  pros::Mutex m_mutex;
  std::array<TelemetryRecord, CAPACITY> m_records = {};
  std::size_t m_oldest = 0;
  std::size_t m_count = 0;
  std::uint32_t m_dropped = 0;
};

}  // namespace cardLib
