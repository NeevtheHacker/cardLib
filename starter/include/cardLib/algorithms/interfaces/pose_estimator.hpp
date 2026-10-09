// pose_estimator.hpp
// Why: lets any localization method (odometry, Kalman, ...) plug into the odometry task.
// Owns: the PoseEstimator interface (SPEC section 18).
#pragma once

#include "cardLib/algorithms/interfaces/robot_state.hpp"
#include "cardLib/algorithms/localization/sensor_frame.hpp"
#include "cardLib/algorithms/math/pose.hpp"

namespace cardLib {

class PoseEstimator {
 public:
  virtual ~PoseEstimator() = default;

  // Called by setPose(), from the odometry task, before the next update.
  virtual void reset(const Pose& startPose) = 0;

  // Called every 10 ms with this loop's readings. Must not allocate.
  virtual RobotState update(const SensorFrame& frame) = 0;
};

}  // namespace cardLib
