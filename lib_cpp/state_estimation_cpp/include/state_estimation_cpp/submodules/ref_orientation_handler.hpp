// Copyright 2023 Marcel Weinmann
#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

// type definitions
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// general constants
#include "tum_helpers_cpp/constants.hpp"

// Param manager
#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

// FIR Filter
#include "classical_filter/fir.hpp"

namespace tam::core::state
{
namespace reforientationhandler
{
// auxilary concept to overload functions for 2D and 3D Filters
template <typename TConfig>
concept HasStateThetaRad = requires {
  {TConfig::STATE_THETA_RAD } -> std::convertible_to<const int&>;
};
}  // namespace reforientationhandler

template <typename TConfig>
class RefOrientationHandler
{
private:
  /**
   * @brief IAC Parameter Manager
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> param_manager_;

  // Variables
  /**
   * @brief calculated reference orientation
   */
  tam::types::control::Odometry reference_orientation;

  /**
   * @brief predicted vehicle velocity from one timesteps ago
   */
  Eigen::Vector<double, TConfig::VEL_MEASUREMENT_VECTOR_SIZE> v_t_minus_one_;

  /**
   * @brief predicted vehicle velocity from two timesteps ago
   */
  Eigen::Vector<double, TConfig::VEL_MEASUREMENT_VECTOR_SIZE> v_t_minus_two_;

  /**
   * @brief FIR filters for the numerical derivative of the vehicle velocity
   */
  std::array<std::unique_ptr<tam::core::state::FIR>, TConfig::VEL_MEASUREMENT_VECTOR_SIZE> filter_;

  /**
   * @brief booleans to properly initialize the numerical differentiation
   */
  bool first_iteration_ = true;
  bool second_iteration_ = true;

public:
  /**
   * @brief Constructor
   */
  RefOrientationHandler();

  /**
   * @brief Update the reference orientation based on the imu measurements and the vehicle odometry
   * 
   * @param[in] x                 - Eigen::Vector:
   *                                output of the last kalman filter step (vehicle state)
   * 
   * @param[in] u                 - Eigen::Vector:
   *                                filtered and averaged imu measurements
   * 
   * @param[out]                  - tam::types::control::Odometry:
   *                                calculated reference angles
   */
  // Function Overload for 2D Filters
  const tam::types::control::Odometry & update(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & x,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u)
    requires (!reforientationhandler::HasStateThetaRad<TConfig>);
  // Function Overload for 3D Filters
  const tam::types::control::Odometry & update(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & x,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u)
    requires (reforientationhandler::HasStateThetaRad<TConfig>);

  /**
   * @brief Get the current status (decide whether the reference angle should be fused)
   *
   * @param[out]                          - tam::types::ErrorLvl:
   *                                        current fusion status
   */
  // Function Overload for 2D Filters
  tam::types::ErrorLvl get_status(void)
  requires (!reforientationhandler::HasStateThetaRad<TConfig>);
  // Function Overload for 3D Filters
  tam::types::ErrorLvl get_status(void)
  requires (reforientationhandler::HasStateThetaRad<TConfig>);


  /**
   * @brief returns a pointer to the param manager
   *
   * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler(void);
};
}  // namespace tam::core::state
#include "state_estimation_cpp/submodules/ref_orientation_handler_impl.hpp"
