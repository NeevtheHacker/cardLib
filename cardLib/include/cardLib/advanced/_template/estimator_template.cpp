// estimator_template.cpp
// Why: the template's do-nothing implementation: it keeps the pose it was reset to.
// Owns: nothing beyond EstimatorTemplate's own state.
#include "cardLib/advanced/_template/estimator_template.hpp"

namespace cardLib {

void EstimatorTemplate::reset(const Pose& startPose) {
  m_state = {};
  m_state.pose = startPose;
  // TODO: reset the algorithm's state (for a Kalman filter, the covariance to P0).
}

RobotState EstimatorTemplate::update([[maybe_unused]] const SensorFrame& frame) {
  // TODO: predict from the wheel deltas and IMU in frame, then correct with the distance
  //       readings. frame.dtSeconds is this loop's measured time step.
  return m_state;
}

}  // namespace cardLib
