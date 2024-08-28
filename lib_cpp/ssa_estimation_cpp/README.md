## Side Slip Angle Estimation C++

This directory contains a modular implementation of a side slip angle estimation which can be used with different input and filter configurations.

The class incorporates multiple subclasses:
- [State Machine](/lib_cpp/ssa_estimation_cpp/include/ssa_estimation_cpp/submodules/state_machine.hpp): Monitors the states of the input modalities and handles which modalities should be fused. This class also returns the overall side slip angle estimation status.
- [UKF](/lib_cpp/ssa_estimation_cpp/include/ssa_estimation_cpp/submodules/ukf.hpp): Unscented Kalman filter implementation, can be exchanged for flexible usage within the state estimation class.

### Side Slip Angle Estimation

The side slip angle estimation predicts the following state vector utilizing inputs from a steering angle sensor, wheelspeed encoders, brake pressure, drive troque, orientation measurement and N IMUs (N is defined [here](/lib_cpp/constants/ssa_estimation_constants/include/ssa_estimation_constants/)):

$$x = \left[\begin{array}{c} {V_COG} \\
	                        {\beta_COG}
\end{array}\right]$$

### State Machine
The following state transitions are defined, necessitating a software reset to exit the ERROR and STALE states:

  *                          |  OK  | WARN | ERROR |
  * ------------------------------------------------
  * any valid_imu_x          |   x  |  x   |       |
  * ------------------------------------------------
  * valid_steering           |   x  |  x   |       |
  * ------------------------------------------------
  * valid_wheelspeed         |   x  |  x   |       |
  * ------------------------------------------------
  * valid_drivetrain_torque  |   x  |      |       |
  * ------------------------------------------------
  * valid_brake_pressure     |   x  |      |       |
  * ------------------------------------------------
  * valid_orientation        |      |      |       |