// black_box.hpp
// Why: writes the last 64 motions to the SD card, only when asked and never during a motion
//      (SPEC section 11).
// Owns: the black box file format and the last save result.
#pragma once

#include "cardLib/chassis/chassis.hpp"

namespace cardLib {

class BlackBox {
 public:
  // The /usd/cardLib folder must exist on the card.
  static constexpr const char* DEFAULT_PATH = "/usd/cardLib/blackbox.csv";

  enum class SaveResult { NotSaved, Saved, Busy, NoCard, OpenFailed };

  explicit BlackBox(Chassis& chassis) : m_chassis(chassis) {}

  // Busy while a motion is running; the robot must be still.
  SaveResult saveToSd(const char* path = DEFAULT_PATH);
  SaveResult lastResult() const { return m_lastResult; }

 private:
  Chassis& m_chassis;
  SaveResult m_lastResult = SaveResult::NotSaved;
};

}  // namespace cardLib
