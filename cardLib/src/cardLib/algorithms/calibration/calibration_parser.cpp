// calibration_parser.cpp
// Why: implements the key=value calibration file reader.
// Owns: nothing; no state.
#include "cardLib/algorithms/calibration/calibration_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace cardLib {

namespace {

constexpr std::size_t MAX_NUMBER_LENGTH = 32;

std::string_view trim(std::string_view text) {
  const auto isSpace = [](char c) { return c == ' ' || c == '\t' || c == '\r'; };
  while (!text.empty() && isSpace(text.front())) {
    text.remove_prefix(1);
  }
  while (!text.empty() && isSpace(text.back())) {
    text.remove_suffix(1);
  }
  return text;
}

// The whole text must be one finite number.
std::optional<double> parseNumber(std::string_view text) {
  if (text.empty() || text.size() >= MAX_NUMBER_LENGTH) {
    return std::nullopt;
  }
  std::array<char, MAX_NUMBER_LENGTH> buffer = {};
  std::copy(text.begin(), text.end(), buffer.begin());
  char* end = nullptr;
  const double value = std::strtod(buffer.data(), &end);
  if (end != buffer.data() + text.size() || !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

// Where a numeric key's value goes, or nullptr if the key isn't a number key.
std::optional<double>* numberSlot(std::string_view key, CalibrationValues& values) {
  struct Slot {
    std::string_view key;
    std::optional<double>* slot;
  };
  const std::array<Slot, 9> slots{{
      {"imuScale", &values.imuScale},
      {"driveScale", &values.driveScale},
      {"effectiveTrackWidthInches", &values.effectiveTrackWidthInches},
      {"verticalWheel1XInches", &values.verticalWheelXInches[0]},
      {"verticalWheel2XInches", &values.verticalWheelXInches[1]},
      {"verticalWheel1DiameterInches", &values.verticalWheelDiameterInches[0]},
      {"verticalWheel2DiameterInches", &values.verticalWheelDiameterInches[1]},
      {"horizontalWheelYInches", &values.horizontalWheelYInches},
      {"horizontalWheelDiameterInches", &values.horizontalWheelDiameterInches},
  }};
  for (const Slot& slot : slots) {
    if (slot.key == key) {
      return slot.slot;
    }
  }
  return nullptr;
}

void parseLine(std::string_view line, CalibrationParse& parse) {
  line = trim(line);
  if (line.empty() || line.front() == '#') {
    return;
  }
  const std::size_t equals = line.find('=');
  if (equals == std::string_view::npos) {
    ++parse.badLines;
    return;
  }
  const std::string_view key = trim(line.substr(0, equals));
  const std::string_view value = trim(line.substr(equals + 1));
  if (key == "date") {
    const std::size_t length = std::min(value.size(), CALIBRATION_DATE_LENGTH - 1);
    std::copy_n(value.begin(), length, parse.values.date.begin());
    parse.values.date[length] = '\0';
    return;
  }
  std::optional<double>* slot = numberSlot(key, parse.values);
  if (slot == nullptr) {
    ++parse.unknownKeys;
    return;
  }
  const std::optional<double> number = parseNumber(value);
  if (!number.has_value()) {
    ++parse.badLines;
    return;
  }
  *slot = number;
}

}  // namespace

CalibrationParse parseCalibrationText(std::string_view text) {
  CalibrationParse parse;
  while (!text.empty()) {
    const std::size_t newline = text.find('\n');
    parseLine(text.substr(0, newline), parse);
    if (newline == std::string_view::npos) {
      break;
    }
    text.remove_prefix(newline + 1);
  }
  return parse;
}

}  // namespace cardLib
