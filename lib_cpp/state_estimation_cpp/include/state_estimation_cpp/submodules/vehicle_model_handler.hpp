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

// Param manager
#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager.hpp"

// TUM helpers
#include "tum_helpers_cpp/numerical.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

namespace tam::core::state
{
template <class TConfig>
class VehicleModelHandler
{
private:
  /**
   * @brief IAC Parameter Manager
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> param_manager_;

  // Variables
  /**
   * @brief calculated vehicle velocity in the body frame
   */
  tam::types::control::Odometry vehicle_odometry_;

  /**
   * @brief sensor status buffer for wheelspeed and steering angle
   */
  tam::types::ErrorLvl wheelspeed_status_ = tam::types::ErrorLvl::STALE;
  tam::types::ErrorLvl steering_angle_status_ = tam::types::ErrorLvl::STALE;

  /**
   * @brief booleans to buffer which vehicle model should be used
   */
  bool kinematic_model_ = false;
  bool non_holonomic_model_ = false;
  bool single_track_model_ = false;

  /**
   * @brief side slip angle calculated from the vehicle model
   */
  double side_slip_angle_ = 0.0;

public:
  /**
   * @brief Constructor
   */
  explicit VehicleModelHandler(const std::string & vehicle_model);

  /**
   * @brief Update the State Estimaion Wheelspeed Odometry input
   * 
   * @param[in] wheel             - tam::types::common::DataPerWheel<double>:
   *                                angular velocity per wheel 
   */
  void input_wheel_angular_velocities(const tam::types::common::DataPerWheel<double> & wheel);

  /**
   * @brief Update the status of the wheelspeed signal
   * 
   * @param[in] status            - tam::types::ErrorLvl:
   *                                Wheelspeed status
   */
  void input_wheelspeed_status(const tam::types::ErrorLvl status);

  /**
   * @brief Update the State Estimaion Wheelspeed Odometry input
   * 
   * @param[in] steering_angle    - double:
   *                                steering angle in rad
   */
  void input_steering_angle(double steering_angle);

  /**
   * @brief Update the status of the steering angle status signal
   * 
   * @param[in] status            - tam::types::ErrorLvl:
   *                                Wheelspeed status
   */
  void input_steering_angle_status(const tam::types::ErrorLvl status);

  /**
   * @brief Get the current linear velocities calculated form the angular velocities of the wheels
   * 
   * @param[out] result           - tam::types::control::Odometry:
   *                                [vx_VelCoG_mps, vy_VelCoG_mps]
   */
  const tam::types::control::Odometry & get_vehicle_model_odometry(void);

  /**
   * @brief Get the current sensor status w.r.t. the vehicle model chosen
   * 
   * @param[out] result           - tam::types::ErrorLvl:
   */
  tam::types::ErrorLvl get_status(void);

  /**
   * @brief returns a pointer to the param manager
   *
   * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler(void);
};
}  // namespace tam::core::state
#include "state_estimation_cpp/submodules/vehicle_model_handler_impl.hpp"
