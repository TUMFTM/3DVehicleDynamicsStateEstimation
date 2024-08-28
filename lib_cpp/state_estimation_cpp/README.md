## State Estimation C++

This directory contains a modular implementation of a state estimation which can be used with different input and filter configurations.

The class incorporates multiple subclasses:
- [Referecence Orientation Handler](/lib_cpp/state_estimation_cpp/include/state_estimation_cpp/submodules/ref_orientation_handler.hpp): Handling of Reference Angles.
- [State Machine](/lib_cpp/state_estimation_cpp/include/state_estimation_cpp/submodules/state_machine.hpp): Monitors the states of the input modalities and handles which modalities should be fused. This class also returns the overall state estimation status.
- [Vehicle Model Handler](/lib_cpp/state_estimation_cpp/include/state_estimation_cpp/submodules/vehicle_model_handler.hpp): Calculates the vehicle velocity based on the Wheelspeed sensor signals or kinematic single track model
- [Filter](/lib_cpp/state_estimation_cpp/include/state_estimation_cpp/filter/kalman_filter/): Various Kalman filter implementations, offering flexible usage within the state estimation class.
- [Helper](/lib_cpp/state_estimation_cpp/include/state_estimation_cpp/filter/helper/outlier_detection.hpp): Outlier Detection to handle signal outliers.

### 2D State Estimation C++

The state estimation predicts the following state vector utilizing inputs from N localization sources, N linear velocity measurements, and N IMUs (N is defined [here](/lib_cpp/constants/state_estimation_constants/include/state_estimation_constants/)):

$$x = \left[\begin{array}{c} {X_m} \\
	                        {Y_m} \\
	                        {\psi} \\
	                        {v_x} \\
	                        {v_y}
\end{array}\right]$$

The implemented Extended Kalman Filter employs a constant velocity model for prediction, selectively incorporating updated measurements. The IMU signals are used directly in the prediction step with the following input vector:

$$u = \left[\begin{array}{c} {\omega_z} \\
	                        {a_x} \\
	                        {a_y}
\end{array}\right]$$

### 3D State Estimation CPP

The state estimation predicts the following state vector utilizing inputs from N localization sources, N linear velocity measurements, and N IMUs (N is defined [here](/lib_cpp/constants/state_estimation_constants/include/state_estimation_constants/)):

$$x = \left[\begin{array}{c} {X_m} \\
	                        {Y_m} \\
	                        {Z_m} \\
	                        {\phi} \\
	                        {\theta} \\
	                        {\psi} \\
	                        {v_x} \\
	                        {v_y} \\
	                        {v_z}
\end{array}\right]$$

The implemented Extended Kalman Filter employs a constant velocity model for prediction, selectively incorporating updated measurements. The IMU signals are used directly in the prediction step with the following input vector:

$$u = \left[\begin{array}{c} {\omega_x} \\
                            {\omega_y} \\
                            {\omega_z} \\
	                        {a_x} \\
	                        {a_y} \\
	                        {a_z}
\end{array}\right]$$

To execute the prediction step, the angular velocity measured by the IMUs must be transformed into a frame perpendicular to the local Cartesian frame.

$$\left[
\begin{array}{c}
	{{\dot{\phi}}} \\
	{{\dot{\theta}}} \\
	{{\dot{\psi}}}
\end{array}\right]=\left[
\begin{array}{c c c}
	{{1}} & {{\sin(\phi)\tan(\theta)}} & {{\cos(\phi)\tan(\theta)}} \\
	{{0}} & {{\cos(\phi)}}             & {{-\sin(\phi)}} \\
	{{0}} & {{\sin(\phi)\sec(\theta)}} & {{\cos(\phi)\sec(\theta)}}
\end{array}\right]\left[
\begin{array}{c}
	{{\omega_x}} \\
	{{\omega_y}} \\
	{{\omega_z}}
\end{array}\right]$$

Similarly, we need to transform the velocity state from the vehicle frame into a frame perpendicular to the local Cartesian frame to obtain the position prediction.

### State Machine

The following state transitions are defined, necessitating a software reset to exit the ERROR and STALE states:

|                       |  OK  | STALE |  WARN  | STALE | ERROR | STALE | ERROR | STALE |
|-----------------------|------|-------|--------|-------|-------|-------|-------|-------|
| any valid_loc_x       |   x  |   x   |    x   |   x   |       |       |       |       |
| any valid_lin_vel_x   |   x  |   x   |        |       |   x   |   x   |       |       |
| any valid_imu_x       |   x  |       |    x   |       |   x   |       |   x   |       |