// calibration_values.hpp
// Why: the measured corrections, kept in code with the date and method of each (SPEC 3, 6).
// Owns: robotCalibration(). Paste CalibrationRoutines::printResults() output here. A file at
//       /usd/cardLib/robot.cal, if present, overrides these keys at startup.
#pragma once

#include "cardLib/api.hpp"

inline cardLib::Calibration robotCalibration() {
  cardLib::Calibration calibration;
  // Not measured yet. Run the calibration routines (SPEC section 6), in order, then replace
  // each line with the printed value and the date it was measured.
  calibration.imuScale = 1.0;
  calibration.driveScale = 1.0;
  calibration.source = "code (not measured)";
  return calibration;
}
