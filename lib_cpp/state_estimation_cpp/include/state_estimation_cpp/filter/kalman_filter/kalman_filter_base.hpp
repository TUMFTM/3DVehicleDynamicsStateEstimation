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

// helper functions
#include "tum_helpers_cpp/geometry/geometry.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

namespace tam::core::state
{
template <class TConfig>
class KFBase
{
public:
  /**
   * @brief Constructor
   */
  KFBase();

  /**
   * @brief update the measurement covariance matrix  
   *
   * @param[in] valid_map     - map that contains the current sensor valid bits
   *                            measurement covariance matrix 
   */
  void update_measurement_covariance_matrix(const std::map<std::string, bool> & valid_map);

  /**
   * @brief update the measurement matrix to only fuse sensors that have been updated
   *
   * @param[in] fusion_vec    - Vector indicating which sensors have been updated 
   *                            (1: update, 0: not updated)
   */
  void update_measurement_matrix(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & fusion_vec);

  /**
   * @brief set the initial valid bits for the sensor status and covariance adaptation
   *
   * @param[in] valid_map     - map that contains the current sensor valid bits
   *                            measurement covariance matrix 
   */
  void set_initial_valid_map(const std::map<std::string, bool> & valid_map);

  /**
   * @brief set the state vector externally to initialize the extended kalman filter
   *
   * @param[in] x             - Vector representing the state vector created from raw measurements
   */
  void set_state_vector(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & x);

  /**
   * @brief sets the position covariance of one input
   *
   * @param[in] covariance        - Eigen::Vector(POS_MEASUREMENT_VECTOR_SIZE):
   *                                covariance of the position measurements
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the position input [0-N]
   */
  void set_position_covariance(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::POS_MEASUREMENT_VECTOR_SIZE>> & covariance,
    uint8_t pos_num);

  /**
   * @brief sets the orientation covariance of one input
   *
   * @param[in] covariance        - Eigen::Vector(ORIENTATION_MEASUREMENT_VECTOR_SIZE):
   *                                covariance of the orientation measurements
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the orientation input [0-N]
   */
  void set_orientation_covariance(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE>>
    & covariance, uint8_t orientation_num);

  /**
   * @brief Get state vector of the Kalman Filter
   * 
   * @param[out]                  - Eigen::Vector:
   *                                Kalman filter state vector
   */
  const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> get_state_vector(void);

  /**
   * @brief Get the predicted covariance matrix of the Kalman Filter
   * 
   * @param[out]                  - Eigen::Matrix:
   *                                Kalman filter covariance matrix
   */
  const Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE>
    get_covariance_matrix(void);

  /**
   * @brief Get pointer on param manager
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler(void);

  // virtual functions
  /**
   * @brief set the covariance matricies defined in the param handler
   */
  virtual void set_covariance_matricies(void) = 0;

  /**
   * @brief Prediction step of the EKF
   *
   * @param[in] u             - Input vector
   */
  virtual void predict(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u) = 0;

  /**
   * @brief Correction step of the EKF
   *
   * @param[in] z             - Measurement vector      
   */
  virtual void update(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & z) = 0;

  /**
   * @brief get all debug values of the kalman filter
   *
   * @param[out]                  - std::map<std::string, double>:
   *                                residuals of the Extended Kalman Filter
   */
  virtual std::map<std::string, double> get_debug(void) = 0;

protected:
  /**
   * @brief transforms measurement covariance matrix (in global cooridnates) 
   *        to a coordinate system along the road 
   *
   * @param[in] M                 - 3x3 Covariance matrix which should be transformed
   * 
   * @param[out]                  - Eigen::Matrix3d:
   *                                transformed covariance matrix
   */
  Eigen::Matrix3d transform_measurement_covariance(const Eigen::Ref<const Eigen::Matrix3d> & M);

  // Variables
  /**
   * @brief State vector of the Kalman Filter
   */
  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_;

  /**
   * @brief State vector of the Kalman Filter after the prediction step
   */
  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_pred_;

  /**
   * @brief Covariance matrix of the Kalman Filter
   */
  Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> P_;

  /**
   * @brief Covariance matrix of the Kalman Filter after the prediction step
   */
  Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> P_pred_;

  /**
   * @brief Measurement noise covariance matrix of the Kalman Filter
   */
  Eigen::Matrix<double, TConfig::MEASUREMENT_VECTOR_SIZE, TConfig::MEASUREMENT_VECTOR_SIZE> R_;

  /**
   * @brief Vector containing the diagonal elements of the adaptive measurement noise covariance matrix
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> R_adaptive_;

  /**
   * @brief Vector containing the number of decay steps needed to be performed on the diagonal elements
   */
  Eigen::Vector<uint16_t, TConfig::MEASUREMENT_VECTOR_SIZE> R_decay_;

  /**
   * @brief Process noise covariance matrix of the Kalman Filter
   */
  Eigen::Matrix<double, TConfig::Q_VECTOR_SIZE, TConfig::Q_VECTOR_SIZE> Q_;

  /**
   * @brief Measurement matrix of the Kalman Filter (only uses fresh measurements)
   */
  Eigen::Matrix<double, TConfig::MEASUREMENT_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> H_;

  /**
   * @brief Full Measurement matrix of the Kalman Filter
   */
  Eigen::Matrix<double, TConfig::MEASUREMENT_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> H_full_;

  /**
   * @brief Kalman Gain
   */
  Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::MEASUREMENT_VECTOR_SIZE> K_;

  /**
   * @brief Innovation/ Measurement Residuals of the Kalman Filter after the outlier rejection
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> residuals_;

  /**
   * @brief Innovation/ Measurement Residuals of the Kalman Filter
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> residuals_raw_;

  /**
   * @brief Maximum residual (everything above this threshold is detected as outlier)
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> outlier_bound_;

  /**
   * @brief Diagonal elements of the mahalanobis covariance matrix
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> mahalanobis_covariance_;

  /**
   * @brief Vector indicating which sensors have been updated and is valid
   *        (1: update, 0: not updated)                 
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> fusion_vec_;

  /**
   * @brief map containing the valid bits of the previous step                
   */
  std::map<std::string, bool> previous_valid_map_;

  /**
   * @brief Pointer on the param manager
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> param_manager_;
};
}  // namespace tam::core::state
#include "state_estimation_cpp/filter/kalman_filter/kalman_filter_base_impl.hpp"
