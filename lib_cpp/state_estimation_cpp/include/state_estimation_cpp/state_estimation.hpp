// Copyright 2023 Marcel Weinmann
#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

// type definitions
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
#include "tum_types_cpp/data_per_wheel.hpp"

// type convertions
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
#include "tum_helpers_cpp/rotations.hpp"

// Param manager
#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager_composer.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

// State Estimation Base Class
#include "state_estimation_base/state_estimation_base.hpp"

// Kalman Filters
#include "state_estimation_cpp/filter/kalman_filter/kalman_filter_base.hpp"
#include "state_estimation_cpp/filter/kalman_filter/ekf.hpp"

// State Estimation State Machine
#include "state_estimation_cpp/submodules/state_machine.hpp"

// Vehicle Model Odometry
#include "state_estimation_cpp/submodules/vehicle_model_handler.hpp"

// IMU Preprocessing
#include "imu_handler/imu_handler.hpp"

// Reference Orientation
#include "state_estimation_cpp/submodules/ref_orientation_handler.hpp"

// Outlier detection helper
#include "state_estimation_cpp/filter/helper/outlier_detection.hpp"

namespace tam::core::state
{
namespace stateestimation
{
// auxilary concept to overload functions for 2D and 3D Filters
template <typename TConfig>
concept HasStateThetaRad = requires {
    {TConfig::STATE_THETA_RAD } -> std::convertible_to<const int&>;
  };
}  // namespace stateestimation

template <typename TConfig>
class StateEstimation : public StateEstimationBase
{
private:
  /**
   * @brief IAC Parameter Manager Composer to combine the parameters of a subclasses
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> param_manager_composer_;

  /**
   * @brief Debug Container to output and publish the debug values of the state machine
   */
  tam::types::common::TUMDebugContainer::SharedPtr state_machine_debug_container_ =
    std::make_shared<tam::types::common::TUMDebugContainer>();

  /**
   * @brief Debug Container to output and publish the debug values of the Kalman filter
   */
  tam::types::common::TUMDebugContainer::SharedPtr kalman_filter_debug_container_ =
    std::make_shared<tam::types::common::TUMDebugContainer>();

  /**
   * @brief State Machine of the State Estimation
   */
  std::shared_ptr<tam::core::state::StateMachine<TConfig>> state_machine_;

  /**
   * @brief Vehicle model class to model the tires and calculate the
   *        linear velocity input for the State Estimation
   */
  std::shared_ptr<tam::core::state::VehicleModelHandler<TConfig>> vehicle_model_handler_;

  /**
   * @brief IMU preprocessing to fuse the IMU measurements and compensate for sensor biases
   */
  std::shared_ptr<tam::core::state::IMUHandler<TConfig>> imu_handler_;

  /**
   * @brief Reference orientation handler to calculate pitch and roll based on the imu measurements
   *        and the predicted vehicle odometry
   */
  std::shared_ptr<tam::core::state::RefOrientationHandler<TConfig>> ref_orientation_handler_;

  /**
   * @brief Kalman Filter of the State Estimation
   */
  std::shared_ptr<tam::core::state::KFBase<TConfig>> kalman_filter_;

  /**
   * @brief Input vector defined in the template structs in:
   *        lib_cpp/constants/state_estimation_constants
   */
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u_;

  /**
   * @brief Raw IMU measurements of input vector defined in the template structs in:
   *        lib_cpp/constants/state_estimation_constants
   */
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE
    * (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> u_raw_;

  /**
   * @brief Measurement vector defined in the template structs in:
   *        lib_cpp/constants/state_estimation_constants
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> z_;

  /**
   * @brief Vector indicating which sensors have been updated 
   *        (1: update, 0: not updated)                 
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> fusion_vec_;

  /**
   * @brief State vector of the Kalman Filter to handle the outputs
   */
  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_out_;

  /**
   * @brief Vector representing the diagonal elements of the measurement covariance matrix to adapt
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> R_vec_adapt_;

  /**
   * @brief Vector buffering the number of consecutive hard outliers of the state estimation inputs
   */
  Eigen::Vector<int, TConfig::NUM_VEL_MEASUREMENT + 2 * (TConfig::NUM_IMU_MEASUREMENT
    + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> outlier_buffer_;

  /**
   * @brief Boolean array indicating whether the asynchronous IMU filter should be stepped
   */
  Eigen::Vector<int, (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>
    updated_acceleration_;
  Eigen::Vector<int, (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>
    updated_angular_velocity_;

  /**
   * @brief Vector containing the road information based on the banking map
   */
  tam::types::common::Vector3D<double> road_angles_ = {0.0, 0.0, 0.0};

  /**
   * @brief Vector containing the road information based on the banking map
   */
  tam::types::common::Vector3D<double> ref_angles_ = {0.0, 0.0, 0.0};

  /**
   * @brief Vector containing the vehicle velocities calculated from the vehicle model
   */
  tam::types::common::Vector3D<double> vehicle_model_velocity_ = {0.0, 0.0, 0.0};

  /**
   * @brief sets the all calculated offsets in the imu_handler
   */
  void set_imu_offsets(void);

public:
  /**
   * @brief Constructor
   */
  explicit StateEstimation(const std::string & vehicle_model);

  /**
   * @brief Steps the state estimation once
   */
  void step(void) override;

  /**
   * @brief Sets the initial state of the state estimation
   * 
   * @param[out] result           - bool:
   *                                True if the initial state was sucessfully set
   */
  bool set_initial_state(void) override;

  // position input
  /**
   * @brief sets the positional input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the odometry measurements of the position input
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the position input [0-N]
   */
  void set_input_position(
    const tam::types::control::Odometry & odometry, uint8_t pos_num) override;

  /**
   * @brief sets the status of the position input
   *
   * @param[in] status            - containing information on the status of the position input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the position input [0-N]
   */
  void set_input_position_status(
    const tam::types::ErrorLvl & status, uint8_t pos_num) override;

  /**
   * @brief sets the status of the position input
   *
   * @param[in] status            - containing information on the status of the position input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the position input [0-N]
   */
  void set_input_position_status(uint8_t status, uint8_t pos_num) override;

  /**
   * @brief forwards a detected timeout in the position input to the state estimation state machine
   *
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the position input [0-N]
   */
  void set_position_timeout(uint8_t pos_num) override;

  // orientation input
  /**
   * @brief sets the orientational input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the odometry measurements of the orientation input
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the orientation input [0-N]
   * @param[in] set_roll          - bool:
   *                                sets the roll input to be fused by the kalman filter
   * @param[in] set_pitch         - bool:
   *                                sets the pitch input to be fused by the kalman filter
   * @param[in] set_yaw           - bool:
   *                                sets the yaw input to be fused by the kalman filter
   */
  void set_input_orientation(
    const tam::types::control::Odometry & odometry, uint8_t orientation_num,
    bool set_roll, bool set_pitch, bool set_yaw) override;

  /**
   * @brief sets the status of the orientation input
   *
   * @param[in] status            - containing information on the status of the orientation input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the orientation input [0-N]
   */
  void set_input_orientation_status(
    const tam::types::ErrorLvl & status, uint8_t orientation_num) override;

  /**
   * @brief sets the status of the orientation input
   *
   * @param[in] status            - containing information on the status of the orientation input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the orientation input [0-N]
   */
  void set_input_orientation_status(uint8_t status, uint8_t orientation_num) override;

  /**
   * @brief forwards a detected timeout in the orientation input to the state estimation state machine
   *
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the orientation input [0-N]
   */
  void set_orientation_timeout(uint8_t orientation_num) override;

  // linear velocity inputs
  /**
   * @brief sets the linear velocity input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the linear velocity measured 
   * @param[in] vel_num           - uint8_t:
   *                                containing the number of the velocity input [0-N]
   */
  void set_input_linear_velocity(
    const tam::types::control::Odometry & odometry, uint8_t vel_num) override;

  /**
   * @brief sets the status of the linear velocity input
   *
   * @param[in] status            - containing information on the status of the linear velocity input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] vel_num           - uint8_t:
   *                                containing the number of the velocity input [0-N]
   */
  void set_input_linear_velocity_status(
    const tam::types::ErrorLvl & status, uint8_t vel_num) override;

  /**
   * @brief sets the status of the linear velocity input
   *
   * @param[in] status            - containing information on the status of the linear velocity input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] vel_num           - uint8_t:
   *                                containing the number of the velocity input [0-N]
   */
  void set_input_linear_velocity_status(uint8_t status, uint8_t vel_num) override;

  /**
   * @brief forwards a detected timeout in the linear velocity input to the state estimation state machine
   *
   * @param[in] vel_num           - uint8_t:
   *                                containing the number of the velocity input [0-N]
   */
  void set_linear_velocity_timeout(uint8_t vel_num) override;

  // IMU inputs
  /**
   * @brief sets the linear acceleration input
   *
   * @param[in] acceleration      - tam::types::control::AccelerationwithCovariances:
   *                                containing the acceleration of a IMU
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_input_acceleration(const tam::types::control::AccelerationwithCovariances & acceleration,
    uint8_t imu_num) override;

  /**
   * @brief sets the angular velocity input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the angular velocity of a IMU
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_input_angular_velocity(
    const tam::types::control::Odometry & odometry, uint8_t imu_num) override;

  /**
   * @brief sets the status of the imu input
   *
   * @param[in] status            - containing information on the status of the imu input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_input_imu_status(const tam::types::ErrorLvl & status, uint8_t imu_num) override;

  /**
   * @brief sets the status of the imu input
   *
   * @param[in] status            - containing information on the status of the imu input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_input_imu_status(uint8_t status, uint8_t imu_num) override;

  /**
   * @brief forwards a detected timeout in the imu input to the state estimation state machine
   *
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_imu_timeout(uint8_t imu_num) override;

  // Wheelspeed input
  /**
   * @brief sets the angular velocity of the wheels as input to convert them into linear velocities
   *
   * @param[in] wheel             - tam::types::common::DataPerWheel<double>:
   *                                angular velocity per wheel 
   */
  void set_input_wheelspeeds(const tam::types::common::DataPerWheel<double> & wheel) override;

  /**
   * @brief sets the status of the wheelspeed input
   *
   * @param[in] status            - containing information on the status of the wheelspeed input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_wheelspeed_status(const tam::types::ErrorLvl & status) override;

  /**
   * @brief sets the status of the wheelspeed input
   *
   * @param[in] status            - containing information on the status of the wheelspeed input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_wheelspeed_status(uint8_t status) override;

  /**
   * @brief forwards a detected timeout in the wheelspeed input to the state estimation state machine
   */
  void set_wheelspeeds_timeout(void) override;

  // Steering angle input
  /**
   * @brief sets the measured steering angle to use it in the vehicle model
   *
   * @param[in] steering_angle    - double:
   *                                steering angle in rad
   */
  void set_input_steering_angle(double steering_angle) override;

  /**
   * @brief sets the status of the steering angle input
   *
   * @param[in] status            - containing information on the status of the steering angle input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_steering_angle_status(const tam::types::ErrorLvl & status) override;

  /**
   * @brief sets the status of the steering angle input
   *
   * @param[in] status            - containing information on the status of the steering angle input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_steering_angle_status(uint8_t status) override;

  /**
   * @brief forwards a detected timeout in the steering angle input to the state estimation state machine
   */
  void set_steering_angle_timeout(void) override;

  // road input
  /**
   * @brief sets the road angle input to compensate the IMU measurements
   *
   * @param[in] road_angles        - tam::types::common::Vector3D<double>:
   *                                  vector containing road angles
   */
  void set_input_road_angles(const tam::types::common::Vector3D<double> & road_angles) override;

  /**
   * @brief sets the status for the road angles obtained from the roadhandler
   *
   * @param[in] status            - containing information on the status of the road angle input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_road_angle_status(const tam::types::ErrorLvl & status) override;

  /**
   * @brief sets the status for the road angles obtained from the roadhandler
   *
   * @param[in] status            - containing information on the status of the road angle input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_road_angle_status(uint8_t status) override;

  // Non state estimation specific output
  /**
   * @brief returns a pointer to the param manager composer
   *
   * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler(void) override;

  // output state estimation
  /**
   * @brief returns the odometry output predicted by the state estimation
   *
   * @param[out]                  - tam::types::control::Odometry
   */
  tam::types::control::Odometry get_odometry(void) override;

  /**
   * @brief returns the linear acceleration output predicted by the state estimation
   *
   * @param[out]                  - tam::types::control::AccelerationwithCovariances
   */
  tam::types::control::AccelerationwithCovariances get_acceleration(void) override;

  /**
   * @brief returns the state estimation status
   *
   * @param[out]                  - tam::types::ErrorLvl
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  tam::types::ErrorLvl get_status(void) override;

  /**
   * @brief returns the state estimation status
   *
   * @param[out]                  - uint8_t:
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  uint8_t get_status_as_uint8_t(void) override;

  /**
   * @brief returns the sideslip angle (Schwimmwinkel) predicted by the state estimation
   *
   * @param[out]                  - double
   */
  double get_sideslip_angle(void) override;

  /**
   * @brief returns a string containing the status message generated by the state machine
   *
   * @param[out]                  - std::string
   */
  std::string get_state_machine_status_msg(void) override;

  /**
   * @brief returns a debug container containing the state machine debug values
   *
   * @param[out]                  - tam::types::common::TUMDebugContainer::SharedPtr
   */
  tam::types::common::TUMDebugContainer::SharedPtr get_state_machine_debug_output(void) override;

  /**
   * @brief returns a debug container containing the kalman filter debug values
   *
   * @param[out]                  - tam::types::common::TUMDebugContainer::SharedPtr
   */
  tam::types::common::TUMDebugContainer::SharedPtr get_kalman_filter_debug_output(void) override;
};
}  // namespace tam::core::state
#include "state_estimation_cpp/state_estimation_impl.hpp"
