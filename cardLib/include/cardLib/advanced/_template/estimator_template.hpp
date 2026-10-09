// estimator_template.hpp
// Why: starting point for your own pose estimator, such as a Kalman filter. Copy this folder,
//      rename EstimatorTemplate, and fill in every TODO (see ../README.md).
// Owns: the estimator's config and state.
#pragma once

#include "cardLib/algorithms/interfaces/pose_estimator.hpp"
#include "cardLib/algorithms/interfaces/robot_state.hpp"
#include "cardLib/algorithms/localization/sensor_frame.hpp"
#include "cardLib/algorithms/math/pose.hpp"

namespace cardLib {

// Every tunable number for this algorithm. Set in robot_config.cpp.
struct EstimatorTemplateConfig {
  // TODO: one named field per parameter (README.md shows what a Kalman filter needs).
};

class EstimatorTemplate : public PoseEstimator {
 public:
  explicit EstimatorTemplate(const EstimatorTemplateConfig& config) : m_config(config) {}

  void reset(const Pose& startPose) override;            // called by setPose()
  RobotState update(const SensorFrame& frame) override;  // every 10 ms; must not allocate

 private:
  EstimatorTemplateConfig m_config;
  RobotState m_state;
  // TODO: algorithm state, sized at compile time.
};

}  // namespace cardLib
