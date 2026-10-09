// calibration_parser_test.cpp
// Why: proves the key=value calibration file is read exactly, and bad input is counted, not used.
// Owns: tests for algorithms/calibration/calibration_parser.
#include "cardLib/algorithms/calibration/calibration_parser.hpp"

#include <cstring>

#include "doctest.h"

using namespace cardLib;

TEST_CASE("every known key is read") {
  const CalibrationParse parse = parseCalibrationText(
      "imuScale=1.009\n"
      "driveScale=0.987\n"
      "effectiveTrackWidthInches=12.1\n"
      "verticalWheel1XInches=-6.02\n"
      "verticalWheel2XInches=5.98\n"
      "verticalWheel1DiameterInches=2.76\n"
      "verticalWheel2DiameterInches=2.74\n"
      "horizontalWheelYInches=-1.5\n"
      "horizontalWheelDiameterInches=2.75\n"
      "date=10/03\n");
  const CalibrationValues& values = parse.values;
  CHECK(parse.badLines == 0);
  CHECK(parse.unknownKeys == 0);
  CHECK(*values.imuScale == doctest::Approx(1.009));
  CHECK(*values.driveScale == doctest::Approx(0.987));
  CHECK(*values.effectiveTrackWidthInches == doctest::Approx(12.1));
  CHECK(*values.verticalWheelXInches[0] == doctest::Approx(-6.02));
  CHECK(*values.verticalWheelXInches[1] == doctest::Approx(5.98));
  CHECK(*values.verticalWheelDiameterInches[0] == doctest::Approx(2.76));
  CHECK(*values.verticalWheelDiameterInches[1] == doctest::Approx(2.74));
  CHECK(*values.horizontalWheelYInches == doctest::Approx(-1.5));
  CHECK(*values.horizontalWheelDiameterInches == doctest::Approx(2.75));
  CHECK(std::strcmp(values.date.data(), "10/03") == 0);
}

TEST_CASE("keys not in the file stay empty, so the code values are kept") {
  const CalibrationParse parse = parseCalibrationText("imuScale=1.01\n");
  CHECK(parse.values.imuScale.has_value());
  CHECK_FALSE(parse.values.driveScale.has_value());
  CHECK_FALSE(parse.values.verticalWheelXInches[0].has_value());
  CHECK(parse.values.date[0] == '\0');
}

TEST_CASE("comments, blank lines, spaces and Windows line endings are fine") {
  const CalibrationParse parse =
      parseCalibrationText("# measured 10/03 by tape\r\n\r\n  driveScale = 0.99 \r\n");
  CHECK(parse.badLines == 0);
  CHECK(*parse.values.driveScale == doctest::Approx(0.99));
}

TEST_CASE("a last line without a newline is still read") {
  CHECK(*parseCalibrationText("imuScale=1.002").values.imuScale == doctest::Approx(1.002));
}

TEST_CASE("unknown keys are counted and ignored") {
  const CalibrationParse parse = parseCalibrationText("wheelSize=3\nimuScale=1.0\n");
  CHECK(parse.unknownKeys == 1);
  CHECK(parse.values.imuScale.has_value());
}

TEST_CASE("bad numbers and lines without '=' are counted, never applied") {
  const CalibrationParse parse =
      parseCalibrationText("imuScale=abc\ndriveScale=1.0x\njust words\ndriveScale=\n");
  CHECK(parse.badLines == 4);
  CHECK_FALSE(parse.values.imuScale.has_value());
  CHECK_FALSE(parse.values.driveScale.has_value());
}

TEST_CASE("a long date is cut to fit") {
  const CalibrationParse parse = parseCalibrationText("date=2026-10-03 after rebuild\n");
  CHECK(std::strlen(parse.values.date.data()) == CALIBRATION_DATE_LENGTH - 1);
}

TEST_CASE("empty text gives nothing") {
  const CalibrationParse parse = parseCalibrationText("");
  CHECK(parse.badLines == 0);
  CHECK_FALSE(parse.values.imuScale.has_value());
}
