// Copyright 2023 Marcel Weinmann
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
#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager.hpp"

namespace tam::core::state
{
class StateEstimationBase
{
public:
  /**
   * @brief Constructor and deconstructor
   */
  StateEstimationBase() = default;
  virtual ~StateEstimationBase() = default;

  /**
   * @brief Steps the state estimation once
   */
  virtual void step(void) = 0;

  /**
   * @brief Sets the initial state of the state estimation
   * 
   * @param[out] result           - bool:
   *                                True if the initial state was sucessfully set
   */
  virtual bool set_initial_state(void) = 0;

  // position input
  /**
   * @brief sets the positional input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the odometry measurements of the localization input
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the localization input [0-N]
   */
  virtual void set_input_position(
    const tam::types::control::Odometry & odometry, uint8_t pos_num) = 0;

  /**
   * @brief sets the status of the localization input
   *
   * @param[in] status            - containing information on the status of the localization input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the localization input [0-N]
   */
  virtual void set_input_position_status(
    const tam::types::ErrorLvl & status, uint8_t pos_num) = 0;

  /**
   * @brief sets the status of the localization input
   *
   * @param[in] status            - containing information on the status of the localization input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the localization input [0-N]
   */
  virtual void set_input_position_status(uint8_t status, uint8_t pos_num) = 0;

  /**
   * @brief forwards a detected timeout in the localization input to the state estimation state machine
   *
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the localization input [0-N]
   */
  virtual void set_position_timeout(uint8_t pos_num) = 0;

  // orientation input
  /**
   * @brief sets the positional input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the odometry measurements of the localization input
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the localization input [0-N]
   * @param[in] set_roll          - bool:
   *                                sets the roll input to be fused by the kalman filter
   * @param[in] set_pitch         - bool:
   *                                sets the pitch input to be fused by the kalman filter
   * @param[in] set_yaw           - bool:
   *                                sets the yaw input to be fused by the kalman filter
   */
  virtual void set_input_orientation(
    const tam::types::control::Odometry & odometry, uint8_t orientation_num,
    bool set_roll, bool set_pitch, bool set_yaw) = 0;

  /**
   * @brief sets the status of the localization input
   *
   * @param[in] status            - containing information on the status of the localization input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the localization input [0-N]
   */
  virtual void set_input_orientation_status(
    const tam::types::ErrorLvl & status, uint8_t orientation_num) = 0;

  /**
   * @brief sets the status of the localization input
   *
   * @param[in] status            - containing information on the status of the localization input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the localization input [0-N]
   */
  virtual void set_input_orientation_status(uint8_t status, uint8_t orientation_num) = 0;

  /**
   * @brief forwards a detected timeout in the localization input to the state estimation state machine
   *
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the localization input [0-N]
   */
  virtual void set_orientation_timeout(uint8_t orientation_num) = 0;

  // linear velocity inputs
  /**
   * @brief sets the linear velocity input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the linear velocity measured 
   * @param[in] vel_num           - uint8_t:
   *                                containing the number of the velocity input [0-N]
   */
  virtual void set_input_linear_velocity(
    const tam::types::control::Odometry & odometry, uint8_t vel_num) = 0;

  /**
   * @brief sets the status of the linear velocity input
   *
   * @param[in] status            - containing information on the status of the linear velocity input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] vel_num           - uint8_t:
   *                                containing the number of the velocity input [0-N]
   */
  virtual void set_input_linear_velocity_status(
    const tam::types::ErrorLvl & status, uint8_t vel_num) = 0;

  /**
   * @brief sets the status of the linear velocity input
   *
   * @param[in] status            - containing information on the status of the linear velocity input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] vel_num           - uint8_t:
   *                                containing the number of the velocity input [0-N]
   */
  virtual void set_input_linear_velocity_status(uint8_t status, uint8_t vel_num) = 0;

  /**
   * @brief forwards a detected timeout in the linear velocity input to the state estimation state machine
   *
   * @param[in] vel_num           - uint8_t:
   *                                containing the number of the velocity input [0-N]
   */
  virtual void set_linear_velocity_timeout(uint8_t vel_num) = 0;

  // IMU inputs
  /**
   * @brief sets the linear acceleration input
   *
   * @param[in] acceleration      - tam::types::control::AccelerationwithCovariances:
   *                                containing the acceleration of a IMU
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  virtual void set_input_acceleration(
    const tam::types::control::AccelerationwithCovariances & acceleration, uint8_t imu_num) = 0;

  /**
   * @brief sets the angular velocity input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the angular velocity of a IMU
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  virtual void set_input_angular_velocity(
    const tam::types::control::Odometry & odometry, uint8_t imu_num) = 0;

  /**
   * @brief sets the status of the imu input
   *
   * @param[in] status            - containing information on the status of the imu input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  virtual void set_input_imu_status(const tam::types::ErrorLvl & status, uint8_t imu_num) = 0;

  /**
   * @brief sets the status of the imu input
   *
   * @param[in] status            - containing information on the status of the imu input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  virtual void set_input_imu_status(uint8_t status, uint8_t imu_num) = 0;

  /**
   * @brief forwards a detected timeout in the imu input to the state estimation state machine
   *
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  virtual void set_imu_timeout(uint8_t imu_num) = 0;

  // Wheelspeed input
  /**
   * @brief sets the angular velocity of the wheels as input to convert them into linear velocities
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
  virtual void set_input_wheelspeed_status(uint8_t status) = 0;

  /**
   * @brief forwards a detected timeout in the wheelspeed input to the state estimation state machine
   */
  virtual void set_wheelspeeds_timeout(void) = 0;

  // Steering angle input
  /**
   * @brief sets the measured steering angle to use it in the vehicle model
   *
   * @param[in] steering_angle    - double:
   *                                steering angle in rad
   */
  virtual void set_input_steering_angle(double steering_angle) = 0;

  /**
   * @brief sets the status of the steering angle input
   *
   * @param[in] status            - containing information on the status of the steering angle input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_steering_angle_status(const tam::types::ErrorLvl & status) = 0;

  /**
   * @brief sets the status of the steering angle input
   *
   * @param[in] status            - containing information on the status of the steering angle input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_steering_angle_status(uint8_t status) = 0;

  /**
   * @brief forwards a detected timeout in the steering angle input to the state estimation state machine
   */
  virtual void set_steering_angle_timeout(void) = 0;

  // Road input
  /**
   * @brief sets the road angle input to compensate the IMU measurements
   *        and improve the estimation of the 3d orientation
   *
   * @param[in] road_angles        - tam::types::common::Vector3D<double>:
   *                                  vector containing road angles
   */
  virtual void set_input_road_angles(
    const tam::types::common::Vector3D<double> & road_angles) = 0;

  /**
   * @brief sets the status for the road angles
   *
   * @param[in] status            - containing information on the status of the road angle input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_road_angle_status(const tam::types::ErrorLvl & status) = 0;

  /**
   * @brief sets the status for the road angles
   *
   * @param[in] status            - containing information on the status of the road angle input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual void set_input_road_angle_status(uint8_t status) = 0;

  // Non state estimation specific output
  /**
   * @brief returns a pointer to the param manager composer
   *
   * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
   */
  virtual std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler(void) = 0;

  // output state estimation
  /**
   * @brief returns the odometry output predicted by the state estimation
   *
   * @param[out]                  - tam::types::control::Odometry
   */
  virtual tam::types::control::Odometry get_odometry(void) = 0;

  /**
   * @brief returns the linear acceleration output predicted by the state estimation
   *
   * @param[out]                  - tam::types::control::AccelerationwithCovariances
   */
  virtual tam::types::control::AccelerationwithCovariances get_acceleration(void) = 0;

  /**
   * @brief returns the state estimation status
   *
   * @param[out]                  - tam::types::ErrorLvl:
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual tam::types::ErrorLvl get_status(void) = 0;

  /**
   * @brief returns the state estimation status
   *
   * @param[out]                  - uint8_t:
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  virtual uint8_t get_status_as_uint8_t(void) = 0;

  /**
   * @brief returns the sideslip angle (Schwimmwinkel) predicted by the state estimation
   *
   * @param[out]                  - double
   */
  virtual double get_sideslip_angle(void) = 0;

  /**
   * @brief returns a string containing the status message generated by the state machine
   *
   * @param[out]                  - std::string
   */
  virtual std::string get_state_machine_status_msg(void) = 0;

  /**
   * @brief returns a debug container containing the state machine debug values
   *
   * @param[out]                  - tam::types::common::TUMDebugContainer::SharedPtr
   */
  virtual tam::types::common::TUMDebugContainer::SharedPtr get_state_machine_debug_output(void) = 0;
  
  /**
   * @brief returns a debug container containing the kalman filter debug values
   *
   * @param[out]                  - tam::types::common::TUMDebugContainer::SharedPtr
   */
  virtual tam::types::common::TUMDebugContainer::SharedPtr get_kalman_filter_debug_output(void) = 0;
};
}  // namespace tam::core::state
