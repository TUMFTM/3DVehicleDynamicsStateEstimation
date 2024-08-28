// Copyright 2024 Sven Goblirsch
#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// type definitions
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// Param manager
#include "param_manager_cpp/param_manager.hpp"
#include "param_manager_cpp/param_manager_base.hpp"

namespace tam::core::ssa
{
class SSAEstimationBase
{
public:
  /**
   * @brief Constructor and deconstructor
   */
  SSAEstimationBase() = default;
  virtual ~SSAEstimationBase() = default;

  /**
   * @brief Steps the ssa estimation once
   */
  virtual void step() = 0;

  // IMU input
  /**
   * @brief sets the linear acceleration input
   *
   * @param[in] acceleration      - tam::types::control::AccelerationwithCovariances:
   *                                containing the acceleration of a IMU
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  virtual void set_input_acceleration(
    const tam::types::control::AccelerationwithCovariances & acceleration,
    const uint8_t imu_num) = 0;

  /**
   * @brief sets the angular velocity input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the angular velocity of an IMU
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  virtual void set_input_angular_velocity(
    const tam::types::control::Odometry & odometry, const uint8_t imu_num) = 0;

  /**
   * @brief sets the status of the imu input
   *
   * @param[in] status            - containing information on the status of the imu input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  virtual void set_input_imu_status(const tam::types::ErrorLvl & status, const uint8_t imu_num) = 0;

  /**
   * @brief sets the status of the imu input
   *
   * @param[in] status            - containing information on the status of the imu input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  virtual void set_input_imu_status(const uint8_t status, const uint8_t imu_num) = 0;

  /**
   * @brief forwards a detected timeout in the imu input to the state machine
   *
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  virtual void set_imu_timeout(const uint8_t imu_num) = 0;

  // Wheelspeed input
  /**
   * @brief sets the angular velocity of the wheels as input to convert them into linear
   velocities
   *
   * @param[in] wheel             - tam::types::common::DataPerWheel<double>:
   *                                angular velocity per wheel
   */
  virtual void set_input_wheelspeeds(const tam::types::common::DataPerWheel<double> & wheel) = 0;

  /**
   * @brief sets the status of the wheelspeed input
   *
   * @param[in] status            - containing information on the status of the wheelspeed input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_wheelspeed_status(const tam::types::ErrorLvl & status) = 0;

  /**
   * @brief sets the status of the wheelspeed input
   *
   * @param[in] status            - containing information on the status of the wheelspeed input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_wheelspeed_status(const uint8_t status) = 0;

  /**
   * @brief forwards a detected timeout in the wheelspeed input to the state machine
   */
  virtual void set_wheelspeed_timeout() = 0;

  // Steering angle input
  /**
   * @brief sets the measured steering angle to use it in the vehicle model
   *
   * @param[in] steering_angle    - double:
   *                                steering angle in rad
   */
  virtual void set_input_steering_angle(const double steering_angle) = 0;

  /**
   * @brief sets the status of the steering angle input
   *
   * @param[in] status            - containing information on the status of the steering angle
   input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_steering_angle_status(const tam::types::ErrorLvl & status) = 0;

  /**
   * @brief sets the status of the steering angle input
   *
   * @param[in] status            - containing information on the status of the steering angle
   input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_steering_angle_status(const uint8_t status) = 0;

  /**
   * @brief forwards a detected timeout in the steering angle input to the state
   * machine
   */
  virtual void set_steering_angle_timeout() = 0;

  // Drivetrain Torque
  /**
   * @brief sets the measured drivetrain torque
   *
   * @param[in] drivetrain_trq_Nm    - double:
   *                                drivetrain torque in Nm
   */
  virtual void set_input_drivetrain_torque(const double drivetrain_trq_Nm) = 0;

  /**
   * @brief sets the status of the drivetrain torque input
   *
   * @param[in] status            - containing information on the status of the drivetrain torque input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_drivetrain_torque_status(const tam::types::ErrorLvl & status) = 0;

  /**
   * @brief sets the status of the drivetrain torque input
   *
   * @param[in] status            - containing information on the status of the drivetrain torque input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_drivetrain_torque_status(const uint8_t status) = 0;

  /**
   * @brief forwards a detected timeout in the drivetrain torque input to the state machine
   * */
  virtual void set_drivetrain_torque_timeout() = 0;

  // Brake Pressure
  /**
   * @brief sets the measured brake pressure per wheel
   *
   * @param[in] wheel             - tam::types::common::DataPerWheel<double>:
   *                                brake pressure per wheel in MPa
   */
  virtual void set_input_brake_pressure(const tam::types::common::DataPerWheel<double> & wheel) = 0;

  /**
   * @brief sets the status of the brake pressure input
   *
   * @param[in] status            - containing information on the status of the brake pressure input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_brake_pressure_status(const tam::types::ErrorLvl & status) = 0;

  /**
   * @brief sets the status of the brake pressure input
   *
   * @param[in] status            - containing information on the status of the brake pressure input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_brake_pressure_status(const uint8_t status) = 0;

  /**
   * @brief forwards a detected timeout in the brake pressure input to the state machine
   * */
  virtual void set_brake_pressure_timeout() = 0;

  /**
   * @brief sets the road angles
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the orientation estimates from the state estimation
   *
   */
  virtual void set_input_vehicle_orientation(const tam::types::control::Odometry & odometry) = 0;

  /**
   * @brief sets the status of the orientation input
   *
   * @param[in] status            - containing information on the status of the orientation input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_orientation_status(const tam::types::ErrorLvl & status) = 0;

  /**
   * @brief sets the status of the orientation input
   *
   * @param[in] status            - containing information on the status of the orientation input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_orientation_status(const uint8_t status) = 0;

  /**
   * @brief forwards a detected timeout in the orientation input to the state
   * machine
   */
  virtual void set_orientation_timeout() = 0;

  // Output
  /**
   * @brief returns a pointer to the param manager composer
   *
   * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
   */
  virtual std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler() = 0;

  /**
   * @brief returns the odometry output predicted by the ssa estimation
   *
   * @param[out]                  - tam::types::control::Odometry
   */
  virtual tam::types::control::Odometry get_odometry() = 0;

  /**
   * @brief returns the ssa estimation status
   *
   * @param[out]                  - tam::types::ErrorLvl:
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual tam::types::ErrorLvl get_status() = 0;

  /**
   * @brief returns the ssa estimation status
   *
   * @param[out]                  - uint8_t:
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual uint8_t get_status_as_uint8_t() = 0;

  /**
   * @brief returns a string containing the status message generated by the state machine
   *
   * @param[out]                  - std::string
   */
  virtual std::string get_state_machine_status_msg() = 0;

  /**
   * @brief returns a debug container containing the state machine debug values
   *
   * @param[out]                  - tam::types::common::TUMDebugContainer::SharedPtr
   */
  virtual tam::types::common::TUMDebugContainer::SharedPtr get_state_machine_debug_output() = 0;

  /**
   * @brief returns a debug container containing the kalman filter debug values
   *
   * @param[out]                  - tam::types::common::TUMDebugContainer::SharedPtr
   */
  virtual tam::types::common::TUMDebugContainer::SharedPtr get_kalman_filter_debug_output() = 0;
};
}  // namespace tam::core::ssa
