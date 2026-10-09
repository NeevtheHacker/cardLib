// calibration_routines.hpp
// Why: runs the SPEC section 6 procedures in order and prints copy-paste C++ for the results.
// Owns: the recorded runs. The math is in algorithms/calibration; sensor readings come from the
//       odometry snapshot; the robot moves only through the chassis.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "cardLib/algorithms/math/angle.hpp"
#include "cardLib/algorithms/math/line_fit.hpp"
#include "cardLib/chassis/chassis.hpp"
#include "cardLib/chassis/odometry_task.hpp"

namespace cardLib {

constexpr std::size_t MAX_CALIBRATION_RUNS = 6;

// The values one calibration number took over the recorded runs (only those measured).
struct RunValues {
  std::array<double, MAX_CALIBRATION_RUNS> values = {};
  std::size_t count = 0;
};

class CalibrationRoutines {
 public:
  static constexpr std::size_t MAX_RUNS = MAX_CALIBRATION_RUNS;
  static constexpr std::size_t MAX_DISTANCE_POINTS = 8;

  explicit CalibrationRoutines(Chassis& chassis) : m_chassis(chassis) {}

  // Marks the start of a run: remembers every sensor's running total.
  void beginRun();

  // Spins in place, slowly, until the IMU has turned this many full turns (blocking).
  // False on timeout.
  bool spin(double turns, TurnDirection direction, std::uint32_t timeoutMs = 60000);

  // 1. IMU scale: after spinning N turns from against a wall and pushing the robot back
  //    against it by hand. Run once each direction.
  void recordImuRun(double turns);

  // 2 and 3. After pushing straight forward along a wall a taped distance (72 in or more).
  void recordPushRun(double tapedInches);

  // 2, horizontal wheel. After pushing straight sideways (to the right) a taped distance.
  void recordSidewaysPushRun(double tapedInches);

  // 4 and 5. After a spin of several turns (use spin()), each direction.
  void recordSpinRun();

  // 6. The distance sensor's current reading against a taped true distance. Three or more.
  bool addDistancePoint(std::size_t sensorIndex, double trueInches);
  std::optional<LineFit> distanceFit(std::size_t sensorIndex) const;

  // The chassis' calibration with every value measured so far replacing the old one.
  Calibration results() const;

  // Prints the results as C++ for calibration_values.hpp, with the pass/fail of each check.
  void printResults() const;

  void clear();

 private:
  struct ImuRuns {
    std::optional<double> clockwiseDegrees;
    std::optional<double> counterclockwiseDegrees;
    double turns = 0.0;
  };

  template <typename T>
  struct Runs {
    std::array<T, MAX_RUNS> values = {};
    std::size_t count = 0;
    void add(const T& value) {
      if (count < MAX_RUNS) {
        values[count++] = value;
      }
    }
  };

  struct SpinRun {
    std::array<std::optional<double>, MAX_VERTICAL_WHEELS> verticalXInches = {};
    std::optional<double> horizontalYInches;
    std::optional<double> effectiveTrackWidthInches;
  };

  struct PushRun {
    std::array<std::optional<double>, MAX_VERTICAL_WHEELS> verticalDiameterInches = {};
    std::optional<double> driveScale;
  };

  struct DistancePoints {
    std::array<double, MAX_DISTANCE_POINTS> raw = {};
    std::array<double, MAX_DISTANCE_POINTS> truth = {};
    std::size_t count = 0;
  };

  struct Measured {
    std::array<RunValues, MAX_VERTICAL_WHEELS> verticalDiameters = {};
    std::array<RunValues, MAX_VERTICAL_WHEELS> verticalOffsets = {};
    RunValues driveScales;
    RunValues horizontalDiameters;
    RunValues horizontalOffsets;
    RunValues trackWidths;
  };

  Measured measure() const;
  SensorTotals changeSinceStart() const;
  double currentVerticalDiameter(std::size_t index) const;
  double currentHorizontalDiameter() const;
  void printValue(const char* name, std::optional<double> value, bool runsAgree) const;

  Chassis& m_chassis;
  SensorTotals m_start = {};
  ImuRuns m_imu;
  Runs<PushRun> m_pushes;
  Runs<double> m_sidewaysDiameters;
  Runs<SpinRun> m_spins;
  std::array<DistancePoints, MAX_DISTANCE_SENSORS> m_distancePoints = {};
};

}  // namespace cardLib
