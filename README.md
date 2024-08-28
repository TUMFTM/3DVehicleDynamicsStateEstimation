<div align="center">

<h1>3DVehicleDynamicsStateEstimation</h1>

[![Linux](https://img.shields.io/badge/os-linux-blue.svg)](https://www.linux.org/)
[![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
[![ROS2humble](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/index.html)

</div>

This repository provides a 3D Point-Mass-based Extended Kalman Filter (EKF) for the estimation of the vehicle pose (position and orientation) and velocity. The approach is extended by additionally fusing the velocity estimates of a Single-Track-Model-based Unscented Kalman Filter (UKF).

## Introduction

Within the field of autonomous driving a reliable pose and velocity estimate has to be ensured at all times. Therefore, a Point-Mass-based EKF is chosen to ensure numerical stability and independence of vehicle and tire parameters. The estimation is seperated in a 2D and a 3D approach. The advantages of a 3D estimate are demonstrated in the associated paper linked below. To improve the estimation, reference angles as well as virtual velocity measurements are introduced. The best results in our study have been achieved by calculating the virtual velocity input with a Single-Track-Model-based UKF. More information about the overall concept as well as a benchmark study can be found in the paper. An overview of the estimation concept is shown in the figure below.

![High Level state estimation concept](/docs/images/repository_structure.png "High Level state estimation concept")

The implemented [state estimation](/lib_cpp/state_estimation_cpp/include/state_estimation_cpp) provides a modular interface for N localization inputs (GNSS or SLAM), N velocity inputs (virtual measurement, optical sensor, wheelspeed encoders) and N IMUs. All classes are templated with configurations using [structs](/lib_cpp/constants/state_estimation_constants/include/state_estimation_constants/) to enable the use of various filter approaches (e.g. 2D, 3D) within the same [node](/packages_cpp/state_estimation_node_cpp/).

The [side slip angle estimation](/lib_cpp/ssa_estimation_cpp/include/ssa_estimation_cpp) is templated as well using [structs](/lib_cpp/constants/ssa_estimation_constants/include/ssa_estimation_constants/). This enables extensions for tire parameter estimation or similar, whilst using the same [node](/packages_cpp/ssa_estimation_node_cpp/).

Both estimators use the [IMU Handler](/lib_cpp/base/imu_handler/include/imu_handler) as well as the [Finite Impulse Response Filter](/lib_cpp/base/classical_filter/include/classical_filter) for bias correction, averaging and prefiltering of the IMU measurements.

## List of components

- config: configurations for 2D-EKF and 3D-EKF State Estimation and UKF Side Slip Angle Estimation
- dependencies
  - msgs: used message definitions
  - param_manager_cpp: package containing a parameter manager for ROS2 (copied from [OPENCarDynamics](https://github.com/TUMFTM/Open-Car-Dynamics/tree/v1.0.0))
  - robot_state_publisher: package to publish static and dynamic transforms (copied from [RobotStatePublisher](https://github.com/ros/robot_state_publisher/tree/3.4.0))
  - ros_debug_helpers_cpp: package to handle debug messages (copied from [OPENCarDynamics](https://github.com/TUMFTM/Open-Car-Dynamics/tree/v1.0.0))
  - ros_param_helpers_cpp: package containing helper functions for the ROS parameter manager (copied from [OPENCarDynamics](https://github.com/TUMFTM/Open-Car-Dynamics/tree/v1.0.0))
  - ros2_watchdog_cpp: package to handle ROS2 messages and monitor their timeouts // implemented by Simon Hoffmann
  - tum_helpers_cpp: custom helper functions // implemented by Simon Hoffmann 
  - tum_type_conversions_ros_cpp: type conversions for custom types and messages // implemented by Phillip Pitschi
  - tum_types_cpp: custom types // implemented by Simon Sagmeister
- example_bag
  - vegas24_semi_pub: ROS-Bag containing an example of two Laps of the TUM Autonomous Motorsports Team on the Las Vegas Motorspeedway
- lib_cpp
  - base:
    - classical_filter: libary containing classical filters (currently implemented: Finite Impulse Response Filter)
    - imu_handler: libary to handle multiple IMU inputs including bias corrections, filtering and averaging
    - ssa_estimation_base: base class for the side slip angle estimation
    - state_estimation_base: base class for the state estimation
  - constants: contains different templates for state and side slip angle estimation to vary the state and measusurement vector
  . ssa_estimation_cpp: base implementation of the side slip angle estimation in native Cpp (no ROS-dependencies)
  - state_estimation_cpp: base implementation of the state estimation in native Cpp (no ROS-dependencies)
- packages_cpp
  - ssa_estimation_node_cpp: ROS2 Node to use the side slip angle estimation
  - state_estimation_node_cpp: ROS2 Node to use the state estimation
  - state_publisher: ROS2 Node to launch the adapted state publisher


## Message definitions

We utilize the following signal names for the ROS2-Nodes. The messages have to be defined accordingly or the implementations of the node subscriptions in /packages_cpp have to be adapted accordingly.

### IMU 
- names: /vehicle/sensor/imu1, /vehicle/sensor/imu2, /vehicle/sensor/imu3
- type: sensor_msgs::msg::Imu
- note: already transformed to COG

### GPS
Odometry:
- names: /vehicle/sensor/odometry1, /vehicle/sensor/odometry2, /vehicle/sensor/odometry3
- type: nav_msgs::msg::Odometry
- note: transformed to COG from its frame_id based on [urdf](/packages_cpp/state_publisher/urdf/urdf.xml)

Status:
- status: /vehicle/sensor/status1, /vehicle/sensor/status2, /vehicle/sensor/status3
- type: diagnostic_msgs::msg::DiagnosticStatus
- note: has to be valid to fuse the GPS input

### Wheelspeed
Wheelspeed:
- name: /vehicle/sensor/wheelspeed_radps
- type: msgs::msg::TUMFloat64PerWheelStamped

Status:
- name: /vehicle/sensor/wheelspeed_status
- type: diagnostic_msgs::msg::DiagnosticStatus

### Steering Angle
- name: /vehicle/sensor/steering_report
- type: msgs::msg::SteeringReport

### Drivetrain Torque
- name: /vehicle/sensor/drivetrain_trq_Nm
- type: std_msgs::msg::Float32

### Brake Pressure
- name: /vehicle/sensor/brake_pressure_Pa
- type: msgs::msg::TUMFloat64PerWheelStamped

## Running the Code

To run the code make sure having ROS2 installed - the code was developed and tested using ROS2-Humble.

### Run locally

Build the required ROS-Nodes using colcon and source the install folder in each used terminal:

```bash
colcon build
source ./install/setup.bash
```

If the build process requires too much RAM, use the following command to limit the load average and thus the number of processes before the build command (for Unix-like operating systems):

```bash
export MAKEFLAGS="-l2.5"
```

The nodes can be launched using the following commands:

State Publisher - Publishes the required transformations from Antenna Position to COG:

```bash
ros2 launch state_publisher state_publisher.py
```

State Estimation:

- 2D:

```bash
ros2 run state_estimation_node_cpp state_estimation_node_cpp_ekf_2d_kin --ros-args --params-file config/state_estimation_cpp_2d_ekf_config.yml
```

- 3D:

```bash
ros2 run state_estimation_node_cpp state_estimation_node_cpp_ekf_3d_kin --ros-args --params-file config/state_estimation_cpp_3d_ekf_config.yml
```

To launch different vehicle models altnerate kin (kinematic) to nh (non-holonomic) or stm (kinematic single track model). Those provide different configurations of the vehicle handler and, thus, different virtual velocity inputs.

Side Slip Angle Estimation:

```bash
ros2 run ssa_estimation_node_cpp ssa_estimation_node --ros-args --params-file config/ssa_estimation_ukf_config.yml
```

Example Bag (note that the msgs have to be sourced in this terminal):

```bash
ros2 bag play vegas24_pub
```

### Docker

Build Dockerimage:

```bash
. docker/build_docker.sh
```

Run Containers:

```bash
docker compose -f docker/compose-file.yml up
```

### Remarks

Note that each estimator has to be tuned accordingly utilizing the config files. The current implementation shows a fine tuning of state and side slip angle estimation for the example bag considering the kinemaic 3D-EKF and the Side Slip Angle Estimator are both launched.
Currently no road angle map is implemented in the published version. Therefore, the 2D-EKF does not show accurate estimations for the provided example bag due to the high banking angles. Addiotional road maps could be fused by adding an additional sensor subscription and utilizing the `update_road_angles(odometry)` function in the state estimation implementation. The status is currently set to Warning as no subscription is implemented. 

To launch the kinematic State Estimation without the Side Slip Angle Estimation the parameter `P_VDC_InitializeWithVelocity` has to be set to false in the config file.

## Related Projects

The implementation of the state_estimation_cpp is mainly based on the following paper by Wischnewski et al.:
- [Vehicle Dynamics State Estimation and Localization for High Performance Race Cars](https://www.sciencedirect.com/science/article/pii/S2405896319303957)

The implementation of the ssa_estimation_cpp is mainly based on the following dissertation of Bechtloff:
- [Schätzung des Schwimmwinkels und fahrdynamischer Parameter zur Verbesserung modellbasierter Fahrdynamikregelungen](https://tuprints.ulb.tu-darmstadt.de/8069/1/2018-01-31_Bechtloff_Jakob.pdf)

## References

If you use the 3DVehicleDynamicsStateEstimation in your work please consider citing our paper:
- [Three-dimensional vehicle dynamics state estimation for high-speed race cars under varying signal quality](https://arxiv.org/abs/2408.14885)

```bibtex
@article{goblirsch2024,
  title={Three-dimensional vehicle dynamics state estimation for high-speed race cars under varying signal quality},
  author={Goblirsch, Sven and Weinmann, Marcel and Betz, Johannes},
  journal={arXiv preprint arXiv:2408.14885},
  year={2024}
}
```

### Contact

- [Sven Goblirsch](mailto:sven.goblirsch@tum.de)
- [Marcel Weinmann](mailto:marcel.weinmann@tum.de)

### Ackknowledgements

We thank the IAC and the TUM Autonomous Motorsports team for their support during the data acquisition and the development of the introduced state estimation. Special thanks to Simon Sagmeister, Maxi Leitenstern, Simon Hoffmann and Dominic Ebner for the provided feedback regarding the implementation of the code.