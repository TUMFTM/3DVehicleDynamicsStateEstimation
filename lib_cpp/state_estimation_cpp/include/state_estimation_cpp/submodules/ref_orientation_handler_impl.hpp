// Copyright 2023 Marcel Weinmann
#pragma once
#include "state_estimation_cpp/submodules/ref_orientation_handler.hpp"

namespace tam::core::state
{
template <typename TConfig> RefOrientationHandler<TConfig>::RefOrientationHandler()
{
  // param_manager
  param_manager_ = std::make_shared<tam::core::ParamManager>();

  param_manager_->declare_parameter(
    "P_VDC_v_dot_filter_coefficients",
    std::vector<double>{0.07722183762197628, 0.24522338986219644, 0.34256086202951863,
    0.24522338986219644, 0.07722183762197628}, tam::types::param::ParameterType::DOUBLE_ARRAY, "");
}

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
template <typename TConfig>
const tam::types::control::Odometry & RefOrientationHandler<TConfig>::update(
    [[maybe_unused]] const Eigen::Ref<const Eigen::Vector<double,
      TConfig::STATE_VECTOR_SIZE>> & x,
    [[maybe_unused]] const Eigen::Ref<const Eigen::Vector<double,
      TConfig::INPUT_VECTOR_SIZE>> & u)
    requires (!reforientationhandler::HasStateThetaRad<TConfig>)
{
  // it does not make sense to compute the reference angles in 2d thats why we just return 0
  return reference_orientation;
}

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
template <typename TConfig>
const tam::types::control::Odometry & RefOrientationHandler<TConfig>::update(
  const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & x,
  const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u)
  requires (reforientationhandler::HasStateThetaRad<TConfig>)
{
  // initialize the numerical differentiation
  if (first_iteration_) {
    v_t_minus_two_ = x.segment(TConfig::STATE_VX_MPS, TConfig::VEL_MEASUREMENT_VECTOR_SIZE);

    // set the Filter Coefficients
    Eigen::VectorXd imu_filter_coefficients = Eigen::Map<Eigen::VectorXd>(
      param_manager_->get_parameter_value("P_VDC_v_dot_filter_coefficients").as_double_array().data(),
      param_manager_->get_parameter_value("P_VDC_v_dot_filter_coefficients").as_double_array().size());

    for (std::size_t i = 0; i < TConfig::VEL_MEASUREMENT_VECTOR_SIZE; ++i) {
        filter_[i] = std::make_unique<tam::core::state::FIR>(imu_filter_coefficients);
    }

    first_iteration_ = false;
    return reference_orientation;
  }
  if (second_iteration_) {
    v_t_minus_one_ = x.segment(TConfig::STATE_VX_MPS, TConfig::VEL_MEASUREMENT_VECTOR_SIZE);
    second_iteration_ = false;
    return reference_orientation;
  }

  // compute the central derivative of the vehicle velocity
  Eigen::Vector<double, TConfig::VEL_MEASUREMENT_VECTOR_SIZE> v_dot;
  v_dot = (x.segment(TConfig::STATE_VX_MPS, TConfig::VEL_MEASUREMENT_VECTOR_SIZE) - v_t_minus_two_)
          / (2 * TConfig::TS);

  // buffer the vehicle velocities for the next cycle
  v_t_minus_two_ = v_t_minus_one_;
  v_t_minus_one_ = x.segment(TConfig::STATE_VX_MPS, TConfig::VEL_MEASUREMENT_VECTOR_SIZE);

  // FIR Filter the velocity derivative
  for (int i = 0; i < TConfig::VEL_MEASUREMENT_VECTOR_SIZE; ++i) {
    v_dot[i] = filter_[i]->step(v_dot[i]);
  }

  // calculate the reference angles base on:
  // https://acl.kaist.ac.kr/wp-content/uploads/2021/10/2013partd_OJW.pdf
  // to ensure that no inf is returned by the asin the values have to be clamped
  double phi_value   = (- v_dot[TConfig::MEASUREMENT_VY_MPS] + u[TConfig::INPUT_AY_MPS2]
                        - u[TConfig::INPUT_DPSI_RADPS] * x[TConfig::STATE_VX_MPS]
                        + u[TConfig::INPUT_DPHI_RADPS] * x[TConfig::STATE_VZ_MPS])
                       / (tam::constants::g_earth * std::cos(x[TConfig::STATE_THETA_RAD]));

  reference_orientation.orientation_rad.x = std::asin(std::clamp(phi_value, -0.5, 0.5));

  double theta_value = (v_dot[TConfig::MEASUREMENT_VX_MPS] - u[TConfig::INPUT_AX_MPS2]
                        - u[TConfig::INPUT_DPSI_RADPS] * x[TConfig::STATE_VY_MPS]
                        + u[TConfig::INPUT_DTHETA_RADPS] * x[TConfig::STATE_VZ_MPS])
                       / tam::constants::g_earth;

  reference_orientation.orientation_rad.y = std::asin(std::clamp(theta_value, -0.5, 0.5));

  return reference_orientation;
}

/**
 * @brief Get the current status (decide whether the reference angle should be fused)
 *
 * @param[out]                          - tam::types::ErrorLvl:
 *                                        current fusion status
 */
template <typename TConfig>
tam::types::ErrorLvl RefOrientationHandler<TConfig>::get_status(void)
requires (!reforientationhandler::HasStateThetaRad<TConfig>)
{
  // because we dont need the reference angles in 2d we always return a error
  return tam::types::ErrorLvl::ERROR;
}

/**
 * @brief Get the current status (decide whether the reference angle should be fused)
 *
 * @param[out]                          - tam::types::ErrorLvl:
 *                                        current fusion status
 */
template <typename TConfig>
tam::types::ErrorLvl RefOrientationHandler<TConfig>::get_status(void)
requires (reforientationhandler::HasStateThetaRad<TConfig>)
{
  if (!first_iteration_ && !second_iteration_) {
    return tam::types::ErrorLvl::OK;
  } else {
    return tam::types::ErrorLvl::ERROR;
  }
}

/**
 * @brief returns a pointer to the param manager
 *
 * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
 */
template <class TConfig> std::shared_ptr<tam::interfaces::ParamManagerBase>
  RefOrientationHandler<TConfig>::get_param_handler(void)
{
  return param_manager_;
}
}  // namespace tam::core::state
