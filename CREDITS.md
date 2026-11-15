# CREDITS.md

Everyone whose work shaped cardLib, whether or not code was copied (SPEC section 17).
If code is ever copied rather than rewritten, its license notice stays in that file.

- **LemLib** (MIT): API style, exit conditions, motion chaining, and hardware abstraction ideas
  from LemLib/hardware; also their V5 Eigen port for `advanced/`. The `Omniwheel` constants
  (`hardware_config.hpp`) are LemLib's approximate measured omni wheel diameters (SPEC 20.2).
- **OkapiLib**: the X-drive convention cardLib follows for every holonomic drive: a positive
  command pushes the robot forward, wheels listed FrontLeft, FrontRight, BackLeft, BackRight
  (SPEC 20.3).
- **EZ-Template**: tracking wheel offset measurement and the IMU drift check idea.
- **5225A**: the position tracking document behind arc odometry.
- **WPILib**: pose exponential, feedforward formulas, PID controller feature set, and the
  wheel-speed desaturation idea behind the holonomic mixer (SPEC 20.8).
- **Brett Beauregard**: "Improving the Beginner's PID".
- **Borenstein and Feng**: UMBmark.
- **Purdue SIGBots**: PROS itself.
- **doctest** (MIT, Viktor Kirilov): the laptop test framework, vendored unchanged in
  `cardLib/tests/third_party/doctest.h` (v2.4.11).
