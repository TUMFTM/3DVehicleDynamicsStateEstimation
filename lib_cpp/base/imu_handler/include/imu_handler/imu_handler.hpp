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

// general constants
#include "tum_helpers_cpp/constants.hpp"

// Param manager
#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"
#include "ssa_estimation_constants/UKF_STM.hpp"

// FIR Filter
#include "classical_filter/fir.hpp"

namespace tam::core::state
{
template <class TConfig>
class IMUHandler
{
private:
  /**
   * @brief IAC Parameter Manager
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> param_manager_;

  // Variables
  /**
   * @brief Vector containing all the sensor biases for the imu signals
   */
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> imu_bias_;

  /**
   * @brief Road angles [banking, slope, 0.0]
   */
  tam::types::common::Vector3D<double> road_angles_;

  /**
   * @brief FIR filters for the imu measurements
   */
  std::array<std::unique_ptr<tam::core::state::FIR>, TConfig::INPUT_VECTOR_SIZE *
    (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> filter_imu_;
  bool initialize_filter_ = true;

  /**
   * @brief Update the raw input vector based on the imu measurements
   * 
   * @param[in] u_raw             - Eigen::Vector:
   *                                vector containing the raw IMU measurements
   * @param[in] u_fusion_vec      - Eigen::Vector:
   *                                vector indicating whether a IMU measurement is valid
   * @param[out] u                - Eigen::Vector:
   *                                Processed input vector to the Kalman Filter
   *                                [dPsi_radps, ax_mps2, ay_mps2]
   */
  const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> average_input_vector(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE *
    (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>> & u_raw,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE *
    (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>> & u_fusion_vec);

public:
  /**
   * @brief Constructor
   */
  IMUHandler();

  /**
   * @brief Update the raw input vector based on the imu measurements
   * 
   * @param[in] u_raw             - Eigen::Vector:
   *                                vector containing the raw IMU measurements
   * @param[in] u_fusion_vec      - Eigen::Vector:
   *                                vector indicating whether a IMU measurement is valid
   * @param[out] u                - Eigen::Vector:
   *                                Processed input vector to the Kalman Filter
   *                                [dPsi_radps, ax_mps2, ay_mps2]
   */
  const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> update_input_vector(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE *
    (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>> & u_raw,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE *
    (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>> & u_fusion_vec);

  /**
   * @brief Update the raw input vector based on the imu measurements
   * 
   * @param[in] u_raw             - Eigen::Vector:
   *                                vector containing the raw IMU measurements
   * @param[out] u_filt           - Eigen::Vector:
   *                                Processed raw input vector to the Kalman Filter
   *                                
   */
  const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> filter_imu_measurements(
      const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u_raw,
      const double imu_num);

  /**
   * @brief set the road angles to compensate banking for 2D acceleration inputs
   * 
   * @param[in] road_angles      - tam::types::common::Vector3D<double>:
   *                                vector containing road angles
   * 
   */
  void set_road_angles(const tam::types::common::Vector3D<double> & road_angles);

  /**
   * @brief set the sensor biases to be compensated in the averaged output vector
   * 
   * @param[in] imu_biases        - Eigen::Vector:
   *                                vector containing biases for all IMU measurements
   */
  void set_sensor_bias(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & imu_bias);

  /**
   * @brief returns a pointer to the param manager
   *
   * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler(void);
};
}  // namespace tam::core::state
#include "imu_handler/imu_handler_impl.hpp"
