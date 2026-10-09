// calibration_file.hpp
// Why: the optional SD calibration file: code values stay the default, the file overrides only
//      the keys it has, and the screen shows where the values came from (SPEC section 3).
// Owns: reading and writing /usd/cardLib/<robot>.cal and the "SD <date>" source text.
#pragma once

#include <cstddef>

#include "cardLib/chassis/chassis_config.hpp"

namespace cardLib {

class CalibrationFile {
 public:
  static constexpr std::size_t MAX_FILE_BYTES = 1024;

  enum class LoadResult { Loaded, NoCard, NotFound, TooLarge };

  // Overrides only the keys in the file, and sets calibration.source to "SD <date>". On any
  // failure calibration is untouched. Apply it with Chassis::setCalibration() before
  // initialize(). badLines reports lines that couldn't be read.
  static LoadResult load(const char* path, Calibration& calibration,
                         std::size_t* badLines = nullptr);

  // Writes every value the calibration has as key=value. False on failure.
  static bool save(const char* path, const Calibration& calibration, const char* date);
};

}  // namespace cardLib
