// calibration_file.cpp
// Why: implements loading and saving the SD calibration file. Startup only, never in a loop.
// Owns: the static "SD <date>" text that calibration.source points to.
#include "cardLib/systems/calibration_file.hpp"

#include <array>
#include <cstdio>
#include <string_view>

#include "cardLib/algorithms/calibration/calibration_parser.hpp"
#include "pros/misc.h"

namespace cardLib {

namespace {

constexpr std::size_t SOURCE_TEXT_LENGTH = 4 + CALIBRATION_DATE_LENGTH;

// calibration.source points here, so it must outlive every Calibration that uses it.
std::array<char, SOURCE_TEXT_LENGTH> sourceText = {};

void applyValues(const CalibrationValues& values, Calibration& calibration) {
  calibration.imuScale = values.imuScale.value_or(calibration.imuScale);
  calibration.driveScale = values.driveScale.value_or(calibration.driveScale);
  if (values.effectiveTrackWidthInches) {
    calibration.effectiveTrackWidthInches = values.effectiveTrackWidthInches;
  }
  for (std::size_t i = 0; i < values.verticalWheelXInches.size(); ++i) {
    if (values.verticalWheelXInches[i]) {
      calibration.verticalWheelXInches[i] = values.verticalWheelXInches[i];
    }
    if (values.verticalWheelDiameterInches[i]) {
      calibration.verticalWheelDiameterInches[i] = values.verticalWheelDiameterInches[i];
    }
  }
  if (values.horizontalWheelYInches) {
    calibration.horizontalWheelYInches = values.horizontalWheelYInches;
  }
  if (values.horizontalWheelDiameterInches) {
    calibration.horizontalWheelDiameterInches = values.horizontalWheelDiameterInches;
  }
}

void writeValue(std::FILE* file, const char* key, const std::optional<double>& value) {
  if (value.has_value()) {
    std::fprintf(file, "%s=%.6f\n", key, *value);
  }
}

}  // namespace

CalibrationFile::LoadResult CalibrationFile::load(const char* path, Calibration& calibration,
                                                  std::size_t* badLines) {
  if (pros::c::usd_is_installed() != 1) {
    return LoadResult::NoCard;
  }
  std::FILE* file = std::fopen(path, "r");
  if (file == nullptr) {
    return LoadResult::NotFound;
  }
  std::array<char, MAX_FILE_BYTES + 1> text = {};
  const std::size_t length = std::fread(text.data(), 1, text.size(), file);
  std::fclose(file);
  if (length > MAX_FILE_BYTES) {
    return LoadResult::TooLarge;
  }

  const CalibrationParse parse = parseCalibrationText(std::string_view(text.data(), length));
  applyValues(parse.values, calibration);
  std::snprintf(sourceText.data(), sourceText.size(), "SD %s", parse.values.date.data());
  calibration.source = sourceText.data();
  if (badLines != nullptr) {
    *badLines = parse.badLines;
  }
  return LoadResult::Loaded;
}

bool CalibrationFile::save(const char* path, const Calibration& calibration, const char* date) {
  if (pros::c::usd_is_installed() != 1) {
    return false;
  }
  std::FILE* file = std::fopen(path, "w");
  if (file == nullptr) {
    return false;
  }
  std::fprintf(file, "# cardLib calibration\n");
  if (date != nullptr) {
    std::fprintf(file, "date=%s\n", date);
  }
  writeValue(file, "imuScale", calibration.imuScale);
  writeValue(file, "driveScale", calibration.driveScale);
  writeValue(file, "effectiveTrackWidthInches", calibration.effectiveTrackWidthInches);
  writeValue(file, "verticalWheel1XInches", calibration.verticalWheelXInches[0]);
  writeValue(file, "verticalWheel2XInches", calibration.verticalWheelXInches[1]);
  writeValue(file, "verticalWheel1DiameterInches", calibration.verticalWheelDiameterInches[0]);
  writeValue(file, "verticalWheel2DiameterInches", calibration.verticalWheelDiameterInches[1]);
  writeValue(file, "horizontalWheelYInches", calibration.horizontalWheelYInches);
  writeValue(file, "horizontalWheelDiameterInches", calibration.horizontalWheelDiameterInches);
  std::fclose(file);
  return true;
}

}  // namespace cardLib
