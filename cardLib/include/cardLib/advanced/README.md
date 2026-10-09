# Adding an advanced algorithm

`advanced/` holds optional algorithms that plug into cardLib's interfaces, such as a Kalman
filter or pure pursuit. You add one by working inside this folder and changing one line of
robot config. Nothing else in cardLib needs editing.

## Steps

1. Copy `_template/` to `advanced/<your_algorithm>/` and rename the class.
2. Pick the interface:
   - `PoseEstimator`: "where is the robot?" (Kalman filter, particle filter)
   - `MotionAlgorithm`: "how should it move?" (pure pursuit, motion profiling)
3. Put **every** tunable number in a `<Name>Config` struct. Nothing hardcoded.
4. Need a sensor reading that `SensorFrame` doesn't have yet? Only two edits:
   1. add the field to `algorithms/localization/sensor_frame.hpp`
   2. fill it in `buildSensorFrame()` in `src/cardLib/chassis/odometry_task.cpp`
5. Add `tests/advanced/<your_algorithm>_test.cpp` that replays a recorded log.
6. Turn it on in `robot_config.cpp`: build it with its config and pass it to the `Chassis`:

   ```cpp
   cardLib::KalmanFilter kalman(kalmanConfig);
   cardLib::Chassis chassis(chassisConfig, &kalman);  // without it: the built-in arc odometry
   ```

7. Based on someone else's work? Add them to `CREDITS.md`.

## Rules

- Never call PROS. An estimator only ever sees a `SensorFrame`, so it can be tested on a laptop
  by replaying logged frames.
- You may use Eigen through LemLib's V5 port (it installs as a PROS template). Nothing in
  `algorithms/` ever does.
- **Fixed-size matrices only**, such as `Eigen::Matrix<double, 9, 9>`. `Eigen::MatrixXd`
  allocates heap memory, which breaks the no-allocation rule.
- Allocate everything in the constructor. `update()` must fit in the 10 ms loop: measure it
  with `pros::micros()` on the robot and log it.
- Errors are return values, never exceptions.
- Every algorithm ships with a laptop test that replays recorded frames.

## What a Kalman filter's config looks like

Every matrix it needs is a named, fixed-size field, so adding one is just adding a line:

```cpp
struct KalmanConfig {
  static constexpr int STATES = 9;        // x, vx, ax, y, vy, ay, theta, omega, alpha
  static constexpr int MEASUREMENTS = 1;  // one distance sensor

  Eigen::Matrix<double, STATES, STATES>             transition;        // F
  Eigen::Matrix<double, STATES, STATES>             processNoise;      // Q
  Eigen::Matrix<double, MEASUREMENTS, STATES>       measurementModel;  // H
  Eigen::Matrix<double, MEASUREMENTS, MEASUREMENTS> measurementNoise;  // R
  Eigen::Matrix<double, STATES, STATES>             startCovariance;   // P0
};
```

The template's `.cpp` lives next to its header so it ships with the library but is never
compiled into it. The laptop tests compile it, so the copy you start from always builds.
