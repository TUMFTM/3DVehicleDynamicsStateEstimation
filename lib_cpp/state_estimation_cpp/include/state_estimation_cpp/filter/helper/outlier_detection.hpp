// Copyright 2023 Marcel Weinmann
#pragma once

#include <eigen3/Eigen/Dense>

// type definitions
#include "tum_types_cpp/common.hpp"

// helper functions
#include "tum_helpers_cpp/rotations.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

namespace tam::core::state::outlier_detection
{
// auxilary concept to overload functions for 2D and 3D Filters
template <typename TConfig>
concept HasStateThetaRad = requires {
  {TConfig::STATE_THETA_RAD } -> std::convertible_to<const int&>;
};

/**
 * @brief Calculate the squared distance between a explict and the mean of a measurement
 *
 * @param[in] measurement       - Explicit measurement
 *
 * @param[in] mean              - Mean of the measurement over window size N
 * 
 * @param[out]                  - Squared distance
 */
Eigen::VectorXd squared_distance(const Eigen::VectorXd & measurement,
                                 const Eigen::VectorXd & mean);
Eigen::Vector3d squared_distance(const tam::types::common::Vector3D<double> & measurement,
                                 const tam::types::common::Vector3D<double> & mean);
Eigen::Vector3d squared_distance(const Eigen::Vector3d & measurement,
                                 const tam::types::common::Vector3D<double> & mean);
/**
 * @brief Performs an outlier detection and rejection using box shaped outlier bounds
 *
 * @param[in] residuals         - Raw residual vector
 *
 * @param[in] outlier_bound     - Box outlier bounds
 * 
 * @param[out]                  - Residual vector without outliers
 */
Eigen::VectorXd box_outlier_detection(const Eigen::VectorXd & residuals,
                                      const Eigen::VectorXd & outlier_bound);

/**
 * @brief Performs an outlier detection and rejection using the mahalanobis distance
 *
 * @param[in] residuals         - Raw residual vector
 *
 * @param[in] covariance_diag   - Diagonal elements of the covariance matrix
 *
 * @param[in] x                 - Current state vector
 * 
 * @param[out]                  - Residual vector without outliers
 */
template <typename TConfig>
Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> mahalanobis_outlier_detection(
  const Eigen::Ref<Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> residuals,
  const Eigen::Ref<Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> covariance_diag,
  const Eigen::Ref<Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> x)
  requires (!HasStateThetaRad<TConfig>);

template <typename TConfig>
Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> mahalanobis_outlier_detection(
  const Eigen::Ref<Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> residuals,
  const Eigen::Ref<Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> covariance_diag,
  const Eigen::Ref<Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> x)
  requires (HasStateThetaRad<TConfig>);

}  // namespace tam::core::state::outlier_detection
#include "state_estimation_cpp/filter/helper/outlier_detection_impl.hpp"
