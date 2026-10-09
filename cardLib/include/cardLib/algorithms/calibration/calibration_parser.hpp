// calibration_parser.hpp
// Why: reads the optional SD calibration file, plain key=value text, without a JSON parser
//      or any allocation (SPEC section 3).
// Owns: the file's key names and parsing rules. No file access; that lives in systems/.
#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string_view>

namespace cardLib {

constexpr std::size_t CALIBRATION_DATE_LENGTH = 16;

// Every value the file may set. Empty means "not in the file; keep the code value".
struct CalibrationValues {
  std::optional<double> imuScale;
  std::optional<double> driveScale;
  std::optional<double> effectiveTrackWidthInches;
  std::array<std::optional<double>, 2> verticalWheelXInches = {};
  std::array<std::optional<double>, 2> verticalWheelDiameterInches = {};
  std::optional<double> horizontalWheelYInches;
  std::optional<double> horizontalWheelDiameterInches;
  std::array<char, CALIBRATION_DATE_LENGTH> date = {};  // "" if not given
};

struct CalibrationParse {
  CalibrationValues values;
  std::size_t badLines = 0;     // no '=', or a value that isn't a whole number
  std::size_t unknownKeys = 0;  // ignored
};

// Lines are key=value; '#' starts a comment line; spaces and \r are ignored. Keys:
// imuScale, driveScale, effectiveTrackWidthInches, verticalWheel1XInches,
// verticalWheel2XInches, verticalWheel1DiameterInches, verticalWheel2DiameterInches,
// horizontalWheelYInches, horizontalWheelDiameterInches, date.
CalibrationParse parseCalibrationText(std::string_view text);

}  // namespace cardLib
