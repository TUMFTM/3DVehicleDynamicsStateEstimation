// Copyright 2023 Marcel Weinmann
#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <type_traits>

// type definitions
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// general constants
#include "tum_helpers_cpp/constants.hpp"

// Param manager
#include "param_manager_cpp/param_manager_base.hpp"

// Kalman Filter Base Class
#include "state_estimation_cpp/filter/kalman_filter/kalman_filter_base.hpp"

// Outlier detection helper
#include "state_estimation_cpp/filter/helper/outlier_detection.hpp"


namespace tam::core::state
{
// Define a helper type trait to test the existence of TConfig::STATE_THETA_RAD
namespace ekf
{
  // auxilary concept to overload functions for 2D and 3D Filters
  template <typename TConfig>
  concept HasStateThetaRad = requires {
    {TConfig::STATE_THETA_RAD } -> std::convertible_to<const int&>;
  };
}  // namespace ekf

template <typename TConfig>
class EKF : public KFBase<TConfig>
{
private:
  // Variables
  /**
   * @brief Jacobian matrix A consisting of partial derivatives of the system update equation
   *        with respect to the system state 
   */
  Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> A_;

  /**
   * @brief Jacobian matrix B consisting of partial derivatives of the system update equation
   *        with respect to the system input
   */
  Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::INPUT_VECTOR_SIZE> B_;

public:
  /**
   * @brief Constructor
   */
  EKF();

  /**
   * @brief set the covariance matricies defined in the param handler
   */
  void set_covariance_matricies(void) override;

  /**
   * @brief Prediction step of the EKF
   *
   * @param[in] u             - Input vector
   */
  void predict(const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u);

  /**
   * @brief Correction step of the EKF
   *
   * @param[in] z             - Measurement vector
   */
  void update(const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & u);

  /**
   * @brief get all debug values of the kalman filter
   *
   * @param[out]              - std::map<std::string, double>:
   *                            residuals of the Extended Kalman Filter
   */
  std::map<std::string, double> get_debug(void);

  /**
   * @brief Tell the compiler that we are using the variables defined in KFBase
   */
  using KFBase<TConfig>::x_;
  using KFBase<TConfig>::x_pred_;
  using KFBase<TConfig>::P_;
  using KFBase<TConfig>::P_pred_;
  using KFBase<TConfig>::R_;
  using KFBase<TConfig>::R_adaptive_;
  using KFBase<TConfig>::Q_;
  using KFBase<TConfig>::H_;
  using KFBase<TConfig>::H_full_;
  using KFBase<TConfig>::K_;
  using KFBase<TConfig>::residuals_;
  using KFBase<TConfig>::residuals_raw_;
  using KFBase<TConfig>::outlier_bound_;
  using KFBase<TConfig>::mahalanobis_covariance_;
  using KFBase<TConfig>::fusion_vec_;
  using KFBase<TConfig>::param_manager_;
};
}  // namespace tam::core::state
#include "state_estimation_cpp/filter/kalman_filter/ekf_impl.hpp"
