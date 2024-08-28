// Copyright 2024 Sven Goblirsch
#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <algorithm>

// type definitions
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// Param manager
#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager.hpp"

// general constants
#include "tum_helpers_cpp/constants.hpp"

// SSA Estimation constants / template input
#include "ssa_estimation_constants/UKF_STM.hpp"

namespace tam::core::ssa
{
template <class TConfig>
class UKF
{
private:
 /**
 * @brief Auxilary variable for initialization
 */
 bool initialize_;

 /**
 * @brief Minimum velocity for the filter to work
 * - below solely the front wheel speed is used to calculate the vehicle velocity
 */
 double vel_min_mps_;

 /**
 * @brief Auxilary variable storing the wheelbase of the vehicle
 */
 double l_m_;

 /**
 * @brief Auxilary variable controlling sigma point weights
 */
 double lambda_;

 /**
 * @brief Auxilary variable storing the longitudinal tire slip
 */
 Eigen::Vector<double, 4> slip_tire_long_perc_;

 /**
 * @brief Auxilary variable storing the lateral tire slip
 */
 Eigen::Vector<double, 4> slip_tire_lat_perc_;

/**
 * @brief Auxilary variable storing the tire loads
 */
Eigen::Vector<double, 4> load_tire_N_;

/**
 * @brief Auxilary variable storing longitudinal, lateral and vertical forces of the front axle
 */
Eigen::Vector<double, 3> force_axle_front_N_;

/**
 * @brief Auxilary variable storing longitudinal, lateral and vertical forces of the front axle
 */
Eigen::Vector<double, 3> force_axle_rear_N_;

public:
 /**
  * @brief Initial time for initialization to ensure correct setup of the covariance matrices
 */
 double init_time_s_ = 0.0;

/**
  * @brief State vector of the Kalman Filter
  */
 Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_;

 /**
  * @brief State vector of the Kalman Filter after the prediction step
  */
 Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_pred_;

 /**
  * @brief Change of the state vector of the Kalman Filter after the update step
  */
 Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_meas_update_;

 /**
 * @brief State vector of the Kalman Filter transformed by H Matrix
 */
 Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE> y_;

 /**
 * @brief Virtual Measurement Vector of the Kalman Filter
 */
 Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE> z_virt_;

 /**
 * @brief State error covariance matrix P 
 */
 Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> P_;

 /**
 * @brief Predicted state error covariance matrix P 
 */
 Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> P_pred_;

 /**
 * @brief Measurement error covariance matrix P 
 */
 Eigen::Matrix<double, TConfig::VIRTMEAS_VECTOR_SIZE, TConfig::VIRTMEAS_VECTOR_SIZE> P_meas_;

 /**
 * @brief Cross covariance matrix P 
 */
 Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::VIRTMEAS_VECTOR_SIZE> P_cross_;

 /**
 * @brief Kalman Gain 
 */
 Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::VIRTMEAS_VECTOR_SIZE> K_Gain_;

 /**
 * @brief Sigma Point mean weight matrix 
 */
 Eigen::VectorXd W_mean_;

 /**
 * @brief Sigma Point covariance weight matrix 
 */
 Eigen::VectorXd W_cov_;

 /**
 * @brief Process noise covariance matrix of the Kalman Filter
 */
 Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> Q_;

 /**
 * @brief Measurement noise covariance matrix of the Kalman Filter
 */
 Eigen::Matrix<double, TConfig::VIRTMEAS_VECTOR_SIZE, TConfig::VIRTMEAS_VECTOR_SIZE> R_;

 /**
 * @brief Predicted SigmaPoints
 */
 Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE,
    2 * TConfig::STATE_VECTOR_SIZE + 1> sigmaPoints_pred_;

 /** 
 * @brief Pointer on the param manager
 */
 std::shared_ptr<tam::interfaces::ParamManagerBase> param_manager_;

/**
 * @brief Constructor
 */
UKF();
/**
 * @brief Initialization of the UKF
 *  
 */
void initialize();

/**
 * @brief Prediction step of the UKF
 * 
 * @param[in] u             - Process input vector
 * 
 */
void predict(const Eigen::Ref<const Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE>> & u);

/**
 * @brief Correction step of the UKF
 * 
 * @param[in] z             - Update vector
 * 
 */
void update(const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z);

/**
 * @brief Composition of the Process Vector for the kalman prediction
 * 
 * @param[in] u_imu             - Vector containing all IMU inputs
 * 
 * @param[in] z_meas            - Vector containing all remaining measurements
 * 
 * @param[out]                  - Eigen::Vector: Vector containing all inputs for the kalman prediction
 * 
 */
 Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE> get_u(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u_imu,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & z_meas);

/**
 * @brief Composition of the Measurement Vector for the kalman update
 * 
 * @param[in] u_imu             - Vector containing all IMU inputs
 * 
 * @param[in] z_meas            - Vector containing all remaining measurements
 * 
 * @param[out]                  - Eigen::Vector: Vector containing all inputs for the kalman correction
 * 
 */
 Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE> get_z(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u_imu,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & z_meas);

/**
 * @brief Adapt Process Noise
 * 
 * @param[in] z                  - Vector containing all measurement inputs for the kalman update
 * 
 */
void adaptR(const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z);

/**
 * @brief process function
 * 
 * @param[in] sigma_point       - calculated sigma point of the current step
 * 
 * @param[in] u                 - Vector containing all measurement inputs for the kalman prediction
 * 
 * @param[out]                  - Eigen::Vector: Vector containing processed sigma point
 * 
 */
 Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> process_function(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & sigma_point,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE>> & u);

/**
* @brief Virtual Measurement Vector of the Kalman Filter
*
* @param[in] z                 - Vector containing all measurement inputs for the kalman correction
*
* @param[out]                  - Eigen::Vector: Vector containing all virtual measurements
*
*/
Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE> get_virtual_measurement(
   const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z);

/**
 * @brief transfer function transforming the sigma point into the shape of the virtual measurement
 * 
 * @param[in] sigma_point       - calculated sigma point of the current step
 * 
 * @param[in] z                 - Vector containing all measurement inputs for the kalman correction
 * 
 * @param[out]                  - Eigen::Vector: Vector containing transformed sigma point
 * 
 */
 Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE> transfer_function(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & sigma_point,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z);


/**
* @brief calculate load and slip conditions of each tire
* 
* @param[in] sigma_point       - calculated sigma point of the current step
* 
* @param[in] z                 - Vector containing all measurement inputs for the kalman correction
*
* @param[out]                  - Eigen::Vector: Vector containing the tire states (long, lat slip)
*
*/
Eigen::Matrix<double, 4, 2> calc_tire_states(
   const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & sigma_point,
   const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z);

/**
* @brief calculate load and slip conditions of each tire
* 
* @param[in] sigma_point       - calculated sigma point of the current step
* 
* @param[in] z                 - Vector containing all measurement inputs for the kalman correction
*
*/
Eigen::Vector<double, 4>  calc_tire_loads(
   const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & sigma_point,
   const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z);

/**
* @brief calculate axle forces in longitudinal, lateral and vertical direction
*
* @param[in] slip_tire_long_perc - calculated longitudinal slip
*
* @param[in] slip_tire_lat_perc - calculated lateral slip
*
* @param[in] load_tire_N       - calculated vertical load
*
* @param[out]                  - Eigen::Vector: Vector containing the axle forces (x, y, z) for front and rear axle
*/
Eigen::Vector<double, 6> calc_axle_forces(
   const Eigen::Ref<const Eigen::Vector<double, 4>> & slip_tire_long_perc,
   const Eigen::Ref<const Eigen::Vector<double, 4>> & slip_tire_lat_perc,
   const Eigen::Ref<const Eigen::Vector<double, 4>> & load_tire_N);

/**
* @brief MF Simple Tire Model
* 
* @param[in] slip               - Double containing wheel slip
* 
* @param[in] load               - Double containing wheel/axle load
*
* @param[in] tire_params        - Vector containing tire parameters
* 
* @param[out]                   - double: calculated tire force acc. MF Simple
*
*/
double MF_Simple(
    const double slip, const double load,
    const Eigen::Ref<const Eigen::Vector<double, 4>> & tire_params);

/**
 * @brief Get state vector of the Kalman Filter
 * 
 * @param[out]                  - Eigen::Vector:
 *                                Kalman filter state vector
 */
Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> get_state_vector();

/**
 * @brief Get linear velocity covariance of the Kalman Filter
 * 
 * @param[out]                  - Eigen::Vector:
 *                                Linear velocity covariance
 */
Eigen::Matrix<double, 2, 2> get_linear_velocity_covariance();

/**
 * @brief Get pointer on param manager
 */
std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler();

/**
 * @brief get all debug values of the kalman filter
 * 
 * @param[out]              - std::map<std::string, double>:
 *                             debug values of the UKF
 */
std::map<std::string, double> get_debug();
};
}  // namespace tam::core::ssa
#include "ssa_estimation_cpp/submodules/ukf_impl.hpp"
