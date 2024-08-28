// Copyright 2024 Sven Goblirsch
#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// type definitions
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
#include "tum_types_cpp/data_per_wheel.hpp"

// type convertions
#include "tum_helpers_cpp/rotations.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"

// Param manager
#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager_composer.hpp"

// SSA Estimation Base Class
#include "ssa_estimation_base/ssa_estimation_base.hpp"

// Template Input
#include "ssa_estimation_constants/UKF_STM.hpp"

// SSA Estimation State Machine
#include "ssa_estimation_cpp/submodules/state_machine.hpp"
#include "ssa_estimation_cpp/submodules/ukf.hpp"

// IMU Preprocessing
#include "imu_handler/imu_handler.hpp"

// FIR Filter
#include "classical_filter/fir.hpp"

namespace tam::core::ssa
{
template <class TConfig>
class SSAEstimation : public SSAEstimationBase
{
private:
  /**
   * @brief IAC Parameter Manager Composer to combine the parameters of all subclasses
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> param_manager_composer_;

  /**
   * @brief State Machine of the Side Slip Angle Estimation
   */
  std::shared_ptr<tam::core::ssa::StateMachine<TConfig>> state_machine_;

  /**
   * @brief IMU preprocessing to fuse the IMU measurements and compensate for sensor biases
   */
  std::shared_ptr<tam::core::state::IMUHandler<TConfig>> imu_handler_;

    /**
   * @brief Kalman Filter of the Side Slip Angle Estimation
   */
  std::shared_ptr<tam::core::ssa::UKF<TConfig>> kalman_filter_;

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
  * @brief State vector of the Kalman Filter to handle the outputs
  */
  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_out_;

  /**
   * @brief IMU input vector defined in the template structs in:
   *        lib_cpp/constants/ssa_estimation_constants
   */
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u_imu_;

  /**
   * @brief Raw IMU measurements of imu input vector defined in the template structs in:
   *        lib_cpp/constants/ssa_estimation_constants
   */
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE *
    (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> u_imu_raw_;

  /**
  * @brief Measurement vector containing additional measurements defined in the template
  *        structs in: lib_cpp/constants/ssa_estimation_constants
  */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> z_meas_;

  /**
  * @brief Vector buffering the number of consecutive hard outliers of the state estimation inputs
  */
  Eigen::Vector<int, 2 * (TConfig::NUM_IMU_MEASUREMENT
    + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> outlier_buffer_;

  /**
  * @brief Boolean array indicating whether the asynchronous IMU filter should be stepped
  */
  Eigen::Vector<double, (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>
    updated_acceleration_;
  Eigen::Vector<double, (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>
    updated_angular_velocity_;

  // Yaw Acceleration
  /**
   * @brief FIR filter the yaw velocity measurement
   */
  std::unique_ptr<tam::core::state::FIR> filter_yaw_rate_radps_;
  bool initialize_filter_ = true;

  /**
   * @brief FIR filter the yaw velocity measurement
   */
  double prev_yaw_rate_radps_{0.0};

  /**
   * @brief current yaw acceleration
   */
  double yaw_acc_radps2_{0.0};

public:
  /**
   * @brief Constructor
   */
  SSAEstimation();

  /**
   * @brief Steps the side slip angle estimation once
   */
  void step() override;

  // IMU inputs
  /**
   * @brief sets the linear acceleration input
   *
   * @param[in] acceleration      - tam::types::control::AccelerationwithCovariances:
   *                                containing the acceleration of a IMU
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_input_acceleration(
    const tam::types::control::AccelerationwithCovariances & acceleration,
    const uint8_t imu_num) override;

  /**
   * @brief sets the angular velocity input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the angular velocity of a IMU
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_input_angular_velocity(const tam::types::control::Odometry & odometry,
    const uint8_t imu_num) override;

  /**
   * @brief sets the status of the imu input
   *
   * @param[in] status            - containing information on the status of the imu input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_input_imu_status(const tam::types::ErrorLvl & status, const uint8_t imu_num) override;

  /**
   * @brief sets the status of the imu input
   *
   * @param[in] status            - containing information on the status of the imu input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_input_imu_status(const uint8_t status, const uint8_t imu_num) override;

  /**
   * @brief forwards a detected timeout in the imu input to the state machine
   *
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_imu_timeout(const uint8_t imu_num) override;

  // Wheelspeed input
  /**
   * @brief sets the angular velocity of the wheels as input to convert them into linear
   velocities
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
  void set_input_wheelspeed_status(const uint8_t status) override;

  /**
   * @brief forwards a detected timeout in the wheelspeed input to the state
   * machine
   */
  void set_wheelspeed_timeout() override;

  // Steering angle input
  /**
   * @brief sets the measured steering angle
   *
   * @param[in] steering_angle    - double:
   *                                steering angle in rad
   */
  void set_input_steering_angle(const double steering_angle) override;

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
   * */
  void set_input_steering_angle_status(const uint8_t status) override;

  /**
   * @brief forwards a detected timeout in the steering angle input to the state machine
   * */
  void set_steering_angle_timeout() override;

  // Drivetrain Torque
  /**
   * @brief sets the measured drivetrain torque
   *
   * @param[in] drivetrain_trq_Nm    - double:
   *                                drivetrain torque in Nm
   */
  void set_input_drivetrain_torque(const double drivetrain_trq_Nm) override;

  /**
   * @brief sets the status of the drivetrain torque input
   *
   * @param[in] status            - containing information on the status of the drivetrain torque input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_drivetrain_torque_status(const tam::types::ErrorLvl & status) override;

  /**
   * @brief sets the status of the drivetrain torque input
   *
   * @param[in] status            - containing information on the status of the drivetrain torque input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_drivetrain_torque_status(const uint8_t status) override;

  /**
   * @brief forwards a detected timeout in the drivetrain torque input to the state machine
   * */
  void set_drivetrain_torque_timeout() override;

  // Brake Pressure
  /**
   * @brief sets the measured brake pressure per wheel
   *
   * @param[in] wheel             - tam::types::common::DataPerWheel<double>:
   *                                brake pressure per wheel in MPa
   */
  void set_input_brake_pressure(const tam::types::common::DataPerWheel<double> & wheel) override;

  /**
   * @brief sets the status of the brake pressure input
   *
   * @param[in] status            - containing information on the status of the brake pressure input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_brake_pressure_status(const tam::types::ErrorLvl & status) override;

  /**
   * @brief sets the status of the brake pressure input
   *
   * @param[in] status            - containing information on the status of the brake pressure input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_brake_pressure_status(const uint8_t status) override;

  /**
   * @brief forwards a detected timeout in the brake pressure input to the state machine
   * */
  void set_brake_pressure_timeout() override;

  /**
   * @brief sets the road angles
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the orientation estimates from the state estimation
   *
   */
  void set_input_vehicle_orientation(const tam::types::control::Odometry & odometry) override;

  /**
   * @brief sets the status of the orientation input
   *
   * @param[in] status            - containing information on the status of the orientation input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_orientation_status(const tam::types::ErrorLvl & status);

  /**
   * @brief sets the status of the orientation input
   *
   * @param[in] status            - containing information on the status of the orientation input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_orientation_status(const uint8_t status);

  /**
   * @brief forwards a detected timeout in the orientation input to the state machine
   */
  void set_orientation_timeout();

  /**
   * @brief returns a pointer to the param manager composer
   *
   * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler() override;

  // output side slip angle estimation
  /**
   * @brief returns the odometry output predicted by the side slip angle estimation
   *
   * @param[out]                  - tam::types::control::Odometry
   */
  tam::types::control::Odometry get_odometry() override;

  /**
   * @brief returns the side slip angle estimation status
   *
   * @param[out]                  - tam::types::ErrorLvl
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  tam::types::ErrorLvl get_status() override;

  /**
   * @brief returns the side slip angle estimation status
   *
   * @param[out]                  - uint8_t:
   *                               (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  uint8_t get_status_as_uint8_t() override;

  /**
   * @brief returns a string containing the status message generated by the state machine
   *
   * @param[out]                  - std::string
   */
  std::string get_state_machine_status_msg() override;

  /**
   * @brief returns a debug container containing the state machine debug values
   *
   * @param[out]                  - tam::types::common::TUMDebugContainer::SharedPtr
   */
  tam::types::common::TUMDebugContainer::SharedPtr get_state_machine_debug_output() override;

  /**
   * @brief returns a debug container containing the kalman filter debug values
   *
   * @param[out]                  - tam::types::common::TUMDebugContainer::SharedPtr
   */
  tam::types::common::TUMDebugContainer::SharedPtr get_kalman_filter_debug_output() override;
};
}  // namespace tam::core::ssa
#include "ssa_estimation_cpp/ssa_estimation_impl.hpp"
