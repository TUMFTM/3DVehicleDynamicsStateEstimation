// Copyright 2023 Marcel Weinmann
#pragma once
#include "state_estimation_cpp/filter/kalman_filter/kalman_filter_base.hpp"
namespace tam::core::state
{
template <class TConfig> KFBase<TConfig>::KFBase()
{
  // ensure that all matrices are set to zero
  x_.setZero();
  x_pred_.setZero();
  P_.setZero();
  P_pred_.setZero();
  R_.setZero();
  R_adaptive_.setZero();
  R_decay_.setZero();
  Q_.setZero();
  H_.setZero();
  H_full_.setZero();
  K_.setZero();
  residuals_.setZero();
  residuals_raw_.setZero();
  outlier_bound_.setZero();
  mahalanobis_covariance_.setZero();
  fusion_vec_.setZero();

  // param_manager
  param_manager_ = std::make_shared<tam::core::ParamManager>();

  // declare Kalman filter parameters
  param_manager_->declare_parameter(
    "P_VDC_EnableMeasCovAdaptation_EKF", true, tam::types::param::ParameterType::BOOL, "");
  param_manager_->declare_parameter(
    "P_VDC_EnableInputCrossCorrelation", true, tam::types::param::ParameterType::BOOL, "");
  param_manager_->declare_parameter(
    "P_VDC_EnableMahalanobisOutlierDetection", false, tam::types::param::ParameterType::BOOL, "");

  // declare covariance adaption parameters
  param_manager_->declare_parameter(
    "P_VDC_MeasC_lim", 1.5, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_MeasC_decay", 0.00165, tam::types::param::ParameterType::DOUBLE, "");
}

/**
 * @brief update the measurement covariance matrix  
 *
 * @param[in] valid_map     - map that contains the current sensor valid bits
 *                            measurement covariance matrix 
 */
template <class TConfig> void
  KFBase<TConfig>::update_measurement_covariance_matrix(
    const std::map<std::string, bool> & valid_map)
{
  // get the configured Measurement covarinace matrix
  Eigen::VectorXd R_configured = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_MeasCov").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_MeasCov").as_double_array().size());

  for (int i = 0; i < R_adaptive_.size(); ++i) {
    double covariance_limit = param_manager_->get_parameter_value("P_VDC_MeasC_lim").as_double();
    double covariance_decay = param_manager_->get_parameter_value("P_VDC_MeasC_decay").as_double();
    if (i >= TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION) {
      covariance_limit /= 100;
      covariance_decay /= 5;
    }
    // ignore the covariance_limit for all SLAM inputs
    if (i < TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
        && i >= (TConfig::POS_MEASUREMENT_VECTOR_SIZE * (TConfig::NUM_POS_MEASUREMENT - 1))) {
      covariance_decay *= 100;
    }
    // ensure no elements are set to zero
    if (std::abs(R_adaptive_(i)) <  R_configured(i)) {
      R_adaptive_(i) = R_configured(i);
    }
    // decrease the variance that was set to the limit after a change in the valid bits
    if (R_(i, i) > R_adaptive_[i]) {
      if (R_(i, i) > covariance_limit && R_adaptive_[i] < covariance_limit) {
        R_(i, i) = covariance_limit;
        R_decay_[i] = static_cast<uint16_t>(covariance_limit / covariance_decay);
      }
      if (((R_(i, i) < covariance_limit || R_adaptive_[i] < covariance_limit)
          && R_(i, i) > covariance_decay * 2) && R_decay_[i] > 0) {
        R_(i, i) -= covariance_decay;
        R_decay_[i]--;
      } else {
        R_(i, i) = R_adaptive_[i];
      }
    } else {
      R_(i, i) = R_adaptive_[i];
    }
  }

  // check all position valid bits
  for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
    std::string key = "valid_pos_" + std::to_string(i + 1);
    if (valid_map.at(key) != previous_valid_map_.at(key)) {
      // set all measurement covariances to the limit if a valid bit changes
      R_(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_X_M,
         i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_X_M)
        = param_manager_->get_parameter_value("P_VDC_MeasC_lim").as_double();

      R_(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Y_M,
         i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Y_M)
        = param_manager_->get_parameter_value("P_VDC_MeasC_lim").as_double();

      if constexpr(TConfig::POS_MEASUREMENT_VECTOR_SIZE > 2) {
        R_(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Z_M,
           i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Z_M)
          = param_manager_->get_parameter_value("P_VDC_MeasC_lim").as_double();
      }
    }
  }

  // check all orientation valid bits
  for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
    std::string key = "valid_orientation_" + std::to_string(i + 1);
    if (valid_map.at(key) != previous_valid_map_.at(key)) {
      // set all measurement covariances to the limit if a valid bit changes
      R_(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
         + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_PSI_RAD,
         TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
         + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_PSI_RAD)
        = param_manager_->get_parameter_value("P_VDC_MeasC_lim").as_double() / 100;

      if constexpr(TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE > 1) {
        R_(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
           + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_PHI_RAD,
           TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
           + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_PHI_RAD)
          = param_manager_->get_parameter_value("P_VDC_MeasC_lim").as_double() / 100;

        R_(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
           + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_THETA_RAD,
           TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
           + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_PHI_RAD)
          = param_manager_->get_parameter_value("P_VDC_MeasC_lim").as_double() / 100;
      }
    }
  }

  previous_valid_map_ = valid_map;
}

/**
 * @brief update the measurement matrix to only fuse sensors that have been updated
 *
 * @param[in] fusion_vec    - Vector indicating which sensors have been updated 
 *                            (1: update, 0: not updated)
 */
template <class TConfig> void
  KFBase<TConfig>::update_measurement_matrix(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & fusion_vec)
{
  // extraction matrix which only uses the sensors configured and updated
  Eigen::Matrix<double, TConfig::MEASUREMENT_VECTOR_SIZE, TConfig::MEASUREMENT_VECTOR_SIZE> H_ext
    = fusion_vec.asDiagonal();

  // buffer the fusion vector for the update step of the filter
  fusion_vec_ = fusion_vec;

  // update the measurement matrix such that we only use updated measurements in the
  // update step of the Kalman filter
  H_ = H_ext * H_full_;
}

/**
 * @brief set the initial valid bits for the sensor status and covariance adaptation
 *
 * @param[in] valid_map     - map that contains the current sensor valid bits
 *                            measurement covariance matrix 
 */
template <class TConfig> void
  KFBase<TConfig>::set_initial_valid_map(
    const std::map<std::string, bool> & valid_map)
{
  // set the initial valid map used for the covariance adaption
  previous_valid_map_ = valid_map;
}

/**
 * @brief set the state vector externally to initialize the extended kalman filter
 *
 * @param[in] x             - Vector representing the state vector created from raw measurements
 */
template <class TConfig> void tam::core::state::KFBase<TConfig>::set_state_vector(
  const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & x)
{
  // set the state vector of the Kalman Filter
  x_ = x;
}

/**
 * @brief sets the position covariance of one input
 *
 * @param[in] covariance        - Eigen::Vector(POS_MEASUREMENT_VECTOR_SIZE):
 *                                covariance of the position measurements
 * @param[in] pos_num           - uint8_t:
 *                                containing the number of the position input [0-N]
 */
template <class TConfig> void
  KFBase<TConfig>::set_position_covariance(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::POS_MEASUREMENT_VECTOR_SIZE>> & covariance,
    uint8_t pos_num)
{
  // set the variance in the diagonal of the measurement noise covariance matrix for the position
  if (pos_num < TConfig::NUM_POS_MEASUREMENT) {
    if (covariance.array().isNaN().any()) {
        // If any element is NaN, set all elements in the specified segment to zer
        R_adaptive_.segment(pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE,
                            TConfig::POS_MEASUREMENT_VECTOR_SIZE).setZero();
    } else {
        // If no element is NaN, assign the covariance vector to the specified segment
        R_adaptive_.segment(pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE,
                            TConfig::POS_MEASUREMENT_VECTOR_SIZE) = covariance;
    }
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: Position input " + std::to_string(pos_num) + " does not exist");
  }
}

/**
 * @brief sets the orientation covariance of one input
 *
 * @param[in] covariance        - Eigen::Vector(ORIENTATION_MEASUREMENT_VECTOR_SIZE):
 *                                covariance of the orientation measurements
 * @param[in] orientation_num   - uint8_t:
 *                                containing the number of the orientation input [0-N]
 */
template <class TConfig> void
  KFBase<TConfig>::set_orientation_covariance(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE>>
    & covariance, uint8_t orientation_num)
{
  // set the variance in the diagonal of the measurement noise covariance matrix for the orientation
  if (orientation_num < TConfig::NUM_ORIENTATION_MEASUREMENT) {
    if (covariance.array().isNaN().any()) {
        // If any element is NaN, set all elements in the specified segment to zero
        R_adaptive_.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                            + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE,
                            TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE).setZero();
    } else {
        // If no element is NaN, assign the covariance vector to the specified segment
        R_adaptive_.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                            + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE,
                            TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE) = covariance;
    }
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: Position input " + std::to_string(orientation_num) +
        " does not exist");
  }
}

/**
 * @brief Get state vector of the Kalman Filter
 * 
 * @param[out]                  - Eigen::Vector(STATE_VECTOR_SIZE):
 *                                Kalman filter state vector
 */
template <class TConfig> const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>
  KFBase<TConfig>::get_state_vector(void)
{
  return x_;
}

/**
 * @brief Get the predicted covariance matrix of the Kalman Filter
 * 
 * @param[out]                  - Eigen::Matrix:
 *                                Kalman filter covariance matrix
 */
template <class TConfig>
const Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE>
  KFBase<TConfig>::get_covariance_matrix(void)
{
  return P_;
}

/**
 * @brief returns a pointer to the param manager
 *
 * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
 */
template <class TConfig> std::shared_ptr<tam::interfaces::ParamManagerBase>
  KFBase<TConfig>::get_param_handler(void)
{
  return param_manager_;
}

/**
 * @brief transforms measurement covariance matrix (in global cooridnates) 
 *        to a coordinate system along the road 
 *
 * @param[in] M                 - 3x3 Covariance matrix which should be transformed
 * 
 * @param[out]                  - Eigen::Matrix3d:
 *                                transformed covariance matrix
 */
template <class TConfig> Eigen::Matrix3d
  KFBase<TConfig>::transform_measurement_covariance(
    const Eigen::Ref<const Eigen::Matrix3d> & M)
{
  // Transformation Matrix T based on the standard rotation matrix
  // uses the convention that 0 degrees heading is east (x-axis)
  Eigen::Matrix3d T;
  T << std::cos(x_[TConfig::STATE_PSI_RAD]), -std::sin(x_[TConfig::STATE_PSI_RAD]), 0,
       std::sin(x_[TConfig::STATE_PSI_RAD]), std::cos(x_[TConfig::STATE_PSI_RAD]), 0,
       0, 0, 1;

  // transform the measurment covariance w.r.t the heading to compensate for different
  // measurement variance in s and d as similarity transform
  return T * M * T.transpose();
}
}  // namespace tam::core::state
