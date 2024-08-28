// Copyright 2023 Marcel Weinmann
#pragma once
#include "state_estimation_cpp/state_estimation.hpp"

namespace tam::core::state
{
template <typename TConfig> StateEstimation<TConfig>::StateEstimation(
  const std::string & vehicle_model)
{
  // initialize all subclasses
  state_machine_ = std::make_unique<tam::core::state::StateMachine<TConfig>>();
  imu_handler_ = std::make_unique<tam::core::state::IMUHandler<TConfig>>();
  ref_orientation_handler_ = std::make_unique<tam::core::state::RefOrientationHandler<TConfig>>();
  vehicle_model_handler_
    = std::make_unique<tam::core::state::VehicleModelHandler<TConfig>>(vehicle_model);

  // initialize kalman filter depending on which template is chosen
  // has to be defined during compile time
  if constexpr(std::is_same_v<TConfig, tam::core::state::EKF_2D> ||
               std::is_same_v<TConfig, tam::core::state::EKF_3D>) {
    // initialize a extended kalman filter
    kalman_filter_ = std::make_unique<tam::core::state::EKF<TConfig>>();
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: Kalman filter could not be initialized - check the template");
  }

  // param_manager
  param_manager_composer_ = std::make_shared<tam::core::ParamManagerComposer>(
    std::vector<std::shared_ptr<tam::interfaces::ParamManagerBase>>{
      state_machine_->get_param_handler(), vehicle_model_handler_->get_param_handler(),
      imu_handler_->get_param_handler(), ref_orientation_handler_->get_param_handler(),
      kalman_filter_->get_param_handler()});

  // Allow the state estimation to initialize its state without a vaild velocity measurement
  param_manager_composer_->declare_parameter(
    "P_VDC_InitializeWithVelocity", true, tam::types::param::ParameterType::BOOL, "");

  // Allow the state estimation to overwrite the state machine if the state covariance
  // for the position is below this threshold
  param_manager_composer_->declare_parameter(
    "P_VDC_SafeCovarianceThreshold", 0.5, tam::types::param::ParameterType::DOUBLE, "");

  // Allow the state estimation to fuse the road angles as orientation measurement
  param_manager_composer_->declare_parameter(
    "P_VDC_FuseRoadAngles", false, tam::types::param::ParameterType::BOOL, "");

  // Allow the state estimation to fuse the reference orientation as orientation measurement
  param_manager_composer_->declare_parameter(
    "P_VDC_FuseRefAngles", true, tam::types::param::ParameterType::BOOL, "");

  // Squared distance threshold used to set the linear velocity input invalid
  param_manager_composer_->declare_parameter(
    "P_VDC_HardLinearVelocityOutlierTH", 10.0, tam::types::param::ParameterType::DOUBLE, "");

  // Squared distance threshold on acceleration input used to set the IMU input invalid
  param_manager_composer_->declare_parameter(
    "P_VDC_HardAccelerometerOutlierTH", 1000.0, tam::types::param::ParameterType::DOUBLE, "");

  // Squared distance threshold on angular velocity input used to set the IMU input invalid
  param_manager_composer_->declare_parameter(
    "P_VDC_HardAngularVelocityOutlierTH", 1.0, tam::types::param::ParameterType::DOUBLE, "");

  // Number of Consecutive hard outliers before changing the State Machine status
  param_manager_composer_->declare_parameter(
    "P_VDC_MaxConsecutiveVelHardOutliers", 250, tam::types::param::ParameterType::INTEGER, "");
  param_manager_composer_->declare_parameter(
    "P_VDC_MaxConsecutiveIMUHardOutliers", 50, tam::types::param::ParameterType::INTEGER, "");

  // set the input and measurent vector to zero
  u_.setZero();
  u_raw_.setZero();
  z_.setZero();
  x_out_.setZero();
  fusion_vec_.setZero();
  outlier_buffer_.setZero();
  updated_acceleration_.setZero();
  updated_angular_velocity_.setZero();
}

/**
 * @brief Steps the state estimation once
 */
template <typename TConfig> void StateEstimation<TConfig>::step()
{
  // preprocess the input vector containing linear accelerations and angular veocities
  // the valid_u_fusion_vec ensures that only valid measurements are fused
  u_ = imu_handler_->update_input_vector(u_raw_, state_machine_->get_valid_u_fusion_vec());

  // update the reference orientation based on the updated imu measurements
  // and the last predicted vehicle odometry
  if (param_manager_composer_->get_parameter_value("P_VDC_FuseRefAngles").as_bool()) {
    // set the output as last orientation measurement
    tam::types::control::Odometry ref_orientation = ref_orientation_handler_->update(x_out_, u_);
    ref_angles_ = ref_orientation.orientation_rad;
    StateEstimation<TConfig>::set_input_orientation(
      ref_orientation, TConfig::NUM_ORIENTATION_MEASUREMENT - 1, true, true, false);
    StateEstimation<TConfig>::set_input_orientation_status(
      ref_orientation_handler_->get_status(), TConfig::NUM_ORIENTATION_MEASUREMENT - 1);
  }

  // update the measurement matrix of the Kalman filter to only fuse updated and valid measurements
  if (param_manager_composer_->get_parameter_value("P_VDC_EnableMeasCovAdaptation_EKF").as_bool()) {
    // update the measurement covariance matrix of the Kalman filter
    kalman_filter_->update_measurement_covariance_matrix(state_machine_->get_debug());
  }

  kalman_filter_->update_measurement_matrix(
    fusion_vec_.cwiseProduct(state_machine_->get_valid_fusion_vec()));

  // buffer the measurement vector to ensure that no asynchronous measurements
  // are received during the prediction step
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> z_sync = z_;

  // set the fusion vector to zero to only consider fresh measurements in the next step
  fusion_vec_.setZero();

  // perform the prediction step of the Kalman filter
  kalman_filter_->predict(u_);

  // perform the update step of the Kalman filter
  kalman_filter_->update(z_sync);

  // get the Kalman state vector
  x_out_ = kalman_filter_->get_state_vector();

  // overwrite the state machine if the state estimation is in a safe state
  if ((kalman_filter_->get_covariance_matrix()(TConfig::STATE_POS_X_M,TConfig::STATE_POS_X_M)
       + kalman_filter_->get_covariance_matrix()(TConfig::STATE_POS_Y_M, TConfig::STATE_POS_Y_M)) / 2
      < param_manager_composer_->get_parameter_value("P_VDC_SafeCovarianceThreshold").as_double()){
    state_machine_->update(true);
  } else {
    state_machine_->update();
  }

  // set the imu offsets
  StateEstimation<TConfig>::set_imu_offsets();
}

/**
 * @brief Sets the initial state of the state estimation
 * 
 * @param[out] result           - bool:
 *                                True if the initial state was sucessfully set
 */
template <typename TConfig> bool StateEstimation<TConfig>::set_initial_state(void)
{
  // state vector to overwrite the original one
  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_overwrite;
  bool input_valid = false;

  // ensure that enough imus are valid to not trigger a safe stop after init
  if (state_machine_->get_num_valid_imus()
      < param_manager_composer_->get_parameter_value("P_VDC_MinValidIMUs").as_int()) {
    return false;
  }

  // to ensure that the initial position is valid the state vector of the kalman filter
  // is only overwritten iff the measurement is valid and updated
  fusion_vec_ = fusion_vec_.cwiseProduct(state_machine_->get_valid_fusion_vec());
  for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
    Eigen::Vector<double, TConfig::POS_MEASUREMENT_VECTOR_SIZE> position_elements
      = fusion_vec_.segment(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE,
                            TConfig::POS_MEASUREMENT_VECTOR_SIZE);

    if (position_elements.isApprox(Eigen::VectorXd::Ones(TConfig::POS_MEASUREMENT_VECTOR_SIZE))) {
        x_overwrite.segment(0, TConfig::POS_MEASUREMENT_VECTOR_SIZE)
          = z_.segment(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE,
                       TConfig::POS_MEASUREMENT_VECTOR_SIZE);
        input_valid = true;
        break;
    }
  }

  Eigen::Vector<double, 2> orientation;
  orientation << road_angles_.x, road_angles_.y;

  // set the orientation in the measurement vector (penultimate measurement)
  z_.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
              + (TConfig::NUM_ORIENTATION_MEASUREMENT - 2)
              * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE, 2) = orientation;

  // set bits in the fusion vector to indicate that a new measurement was received
  fusion_vec_.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                      + (TConfig::NUM_ORIENTATION_MEASUREMENT - 2)
                      * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE, 2)
    = Eigen::VectorXd::Ones(2);

  // set the initial orientation of the state vector iff a valid position was found
  // we have to search for every axis individually because not every
  // orientation measurement contains all axis
  if (input_valid) {
    for (int i = 0; i < TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE; ++i) {
      input_valid = false;
      for (int j = 0; j < TConfig::NUM_ORIENTATION_MEASUREMENT; j++){
        if (fusion_vec_[TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                        + j * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + i] > 0.0) {
          x_overwrite[TConfig::POS_MEASUREMENT_VECTOR_SIZE + i]
            = z_[TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                 + j * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + i];

          input_valid = true;
          break;
        }
      }
      if (!input_valid) {
        break;
      }
    }
  }

  if (param_manager_composer_->get_parameter_value("P_VDC_InitializeWithVelocity").as_bool()) {
    // if a valid position and orientation was found also search for a valid velocity measurement
    if (input_valid) {
      for (int i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
        if (fusion_vec_[TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                        + i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE] > 0.0) {
          x_overwrite.segment(TConfig::POS_MEASUREMENT_VECTOR_SIZE
                              + TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE,
                              TConfig::VEL_MEASUREMENT_VECTOR_SIZE)
            =  z_.segment(i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
                          + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL,
                          TConfig::VEL_MEASUREMENT_VECTOR_SIZE);

          // set the initial state of the kalman filter
          kalman_filter_->set_state_vector(x_overwrite);
          kalman_filter_->set_covariance_matricies();
          kalman_filter_->set_initial_valid_map(state_machine_->get_debug());
          x_out_ = x_overwrite;

          // set the initial state of the State Machine
          // overwrite_state_machine = true allows all state transitions
          state_machine_->update(true);
          return true;
        }
      }
      return false;
    } else {
      return false;
    }
  } else {
    // if the state estimation is allowed to be inititalized without a valid velocity measurement
    // the corresponding indicies in the state vector are set to zero
    if (input_valid) {
      x_overwrite.segment(TConfig::POS_MEASUREMENT_VECTOR_SIZE
                          + TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE,
                          TConfig::VEL_MEASUREMENT_VECTOR_SIZE)
        =  Eigen::VectorXd::Zero(TConfig::VEL_MEASUREMENT_VECTOR_SIZE);

      // set the initial state of the kalman filter
      kalman_filter_->set_state_vector(x_overwrite);
      kalman_filter_->set_covariance_matricies();
      kalman_filter_->set_initial_valid_map(state_machine_->get_debug());
      x_out_ = x_overwrite;

      // set the initial state of the State Machine
      // overwrite_state_machine = true allows all state transitions
      state_machine_->update(true);
      return true;
    } else {
      return false;
    }
  }
}

/**
 * @brief sets the positional input
 *
 * @param[in] odometry          - tam::types::control::Odometry:
 *                                containing the odometry measurements of the position input
 * @param[in] pos_num           - uint8_t:
 *                                containing the number of the position input [0-N]
 */
template <typename TConfig> void StateEstimation<TConfig>::set_input_position(
  const tam::types::control::Odometry & odometry, uint8_t pos_num)
{
  if constexpr (!stateestimation::HasStateThetaRad<TConfig>) {
    // sets the positional input of 2D filters
    // set the measurment vector
    if (pos_num < TConfig::NUM_POS_MEASUREMENT) {
      // set x_Loc_m
      z_[pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE
        + TConfig::MEASUREMENT_POS_X_M] = odometry.position_m.x;
      // set y_Loc_m
      z_[pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE
        + TConfig::MEASUREMENT_POS_Y_M] = odometry.position_m.y;

      // set the bits corresponding to this position input in the fusion_vec to one to fuse them
      // iff the position is also valid in the state machine
      fusion_vec_.segment(pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE,
                          TConfig::POS_MEASUREMENT_VECTOR_SIZE)
        = Eigen::VectorXd::Ones(TConfig::POS_MEASUREMENT_VECTOR_SIZE);

      // update the variance of the sensor to adapt the R matrix of the kalman filter
      if (param_manager_composer_->get_parameter_value("P_VDC_EnableMeasCovAdaptation_EKF").as_bool()) {
        Eigen::Vector<double, TConfig::POS_MEASUREMENT_VECTOR_SIZE> covariance;
        covariance << odometry.pose_covariance[0], odometry.pose_covariance[7];
        kalman_filter_->set_position_covariance(covariance, pos_num);
      }
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: Localization input " + std::to_string(pos_num) + " does not exist");
    }
  } else {
    // sets the positional input 3D filters
      // set the measurment vector
    if (pos_num < TConfig::NUM_POS_MEASUREMENT) {
      // set x_Loc_m
      z_[pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_X_M]
        = odometry.position_m.x;
      // set y_Loc_m
      z_[pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Y_M]
        = odometry.position_m.y;
      // set z_Loc_m
      z_[pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Z_M]
        = odometry.position_m.z;

      // set the bits corresponding to this position input in the fusion_vec to one to fuse them
      // iff the position is also valid in the state machine
      fusion_vec_.segment(pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE,
                          TConfig::POS_MEASUREMENT_VECTOR_SIZE)
        = Eigen::VectorXd::Ones(TConfig::POS_MEASUREMENT_VECTOR_SIZE);

      // update the variance of the sensor to adapt the R matrix of the kalman filter
      if (param_manager_composer_->get_parameter_value("P_VDC_EnableMeasCovAdaptation_EKF").as_bool()) {
        Eigen::Vector<double, TConfig::POS_MEASUREMENT_VECTOR_SIZE> covariance;
        covariance << odometry.pose_covariance[0], odometry.pose_covariance[7],
                      odometry.pose_covariance[14];

        kalman_filter_->set_position_covariance(covariance, pos_num);
      }
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: Position input " + std::to_string(pos_num) + " does not exist");
    }
  }
}

/**
 * @brief sets the status of the position input
 *
 * @param[in] status            - containing information on the status of the position input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 * @param[in] pos_num           - uint8_t:
 *                                containing the number of the position input [0-N]
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_position_status(
  const tam::types::ErrorLvl & status, uint8_t pos_num)
{
  // forward the status of the position input to the state machine
  if (pos_num < TConfig::NUM_POS_MEASUREMENT) {
    state_machine_->set_position_status(status, pos_num);
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: Position input " + std::to_string(pos_num) + " does not exist");
  }
}

/**
 * @brief sets the status of the position input
 *
 * @param[in] status            - containing information on the status of the position input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 * @param[in] pos_num           - uint8_t:
 *                                containing the number of the position input [0-N]
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_position_status(
  uint8_t status, uint8_t pos_num)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  set_input_position_status(tam::type_conversions::error_type_from_diagnostic_level(
    static_cast<unsigned char>(status)), pos_num);
}

/**
 * @brief forwards a detected timeout in the position input to the state estimation state machine
 *
 * @param[in] pos_num           - uint8_t:
 *                                containing the number of the position input [0-N]
 */
template <typename TConfig> void StateEstimation<TConfig>::set_position_timeout(
  uint8_t pos_num)
{
  // set the position input invalid in the state machine
  if (pos_num < TConfig::NUM_POS_MEASUREMENT) {
    state_machine_->set_position_invalid(pos_num);
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: Position input " + std::to_string(pos_num) + " does not exist");
  }
}

/**
 * @brief sets the orientational input
 *
 * @param[in] odometry          - tam::types::control::Odometry:
 *                                containing the odometry measurements of the orientation input
 * @param[in] orientation_num   - uint8_t:
 *                                containing the number of the orientation input [0-N]
 * @param[in] set_roll          - bool:
 *                                sets the roll input to be fused by the kalman filter
 * @param[in] set_pitch         - bool:
 *                                sets the pitch input to be fused by the kalman filter
 * @param[in] set_yaw           - bool:
 *                                sets the yaw input to be fused by the kalman filter
 */
template <typename TConfig> void StateEstimation<TConfig>::set_input_orientation(
  const tam::types::control::Odometry & odometry, uint8_t orientation_num,
  [[maybe_unused]] bool set_roll, [[maybe_unused]] bool set_pitch, bool set_yaw)
{
  if constexpr (!stateestimation::HasStateThetaRad<TConfig>) {
    // set the orientation input 2D filters
    // set the measurment vector
    if (orientation_num < TConfig::NUM_ORIENTATION_MEASUREMENT) {
      // set psi_YawAngleLoc_rad and the corresponding bit in the fusion vector to indicate
      // that the measurement was updated
      if (set_yaw) {
        z_[TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
           + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
           + TConfig::MEASUREMENT_PSI_RAD] = odometry.orientation_rad.z;

        fusion_vec_[TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                    + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
                    + TConfig::MEASUREMENT_PSI_RAD] = 1.0;
      }

      // update the variance of the sensor to adapt the R matrix of the kalman filter
      if (param_manager_composer_->get_parameter_value("P_VDC_EnableMeasCovAdaptation_EKF").as_bool()) {
        Eigen::Vector<double, TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE> covariance;
        covariance << odometry.pose_covariance[35];
        kalman_filter_->set_orientation_covariance(covariance, orientation_num);
      }
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: Orientation input " + std::to_string(orientation_num) + " does not exist");
    }
  } else {
    // set the orientation input of 3D filters
    // set the measurment vector
    if (orientation_num < TConfig::NUM_ORIENTATION_MEASUREMENT) {
      // set phi_RollAngleLoc_rad and the corresponding bit in the fusion vector to indicate
      // that the measurement was updated
      if (set_roll) {
        z_[TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
          + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
          + TConfig::MEASUREMENT_PHI_RAD] = odometry.orientation_rad.x;

        fusion_vec_[TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                    + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
                    + TConfig::MEASUREMENT_PHI_RAD] = 1.0;
      }

      // set theta_PitchAngleLoc_rad and the corresponding bit in the fusion vector to indicate
      // that the measurement was updated
      if (set_pitch) {
        z_[TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
          + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
          + TConfig::MEASUREMENT_THETA_RAD] = odometry.orientation_rad.y;

        fusion_vec_[TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                    + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
                    + TConfig::MEASUREMENT_THETA_RAD] = 1.0;
      }

      // set psi_YawAngleLoc_rad and the corresponding bit in the fusion vector to indicate
      // that the measurement was updated
      if (set_yaw) {
        z_[TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
          + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
          + TConfig::MEASUREMENT_PSI_RAD] = odometry.orientation_rad.z;

        fusion_vec_[TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                    + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
                    + TConfig::MEASUREMENT_PSI_RAD] = 1.0;
      }

      // update the variance of the sensor to adapt the R matrix of the kalman filter
      if (param_manager_composer_->get_parameter_value("P_VDC_EnableMeasCovAdaptation_EKF").as_bool()) {
        Eigen::Vector<double, TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE> covariance;
        covariance << odometry.pose_covariance[21], odometry.pose_covariance[28],
                      odometry.pose_covariance[35];
        kalman_filter_->set_orientation_covariance(covariance, orientation_num);
      }
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: Orientation input " + std::to_string(orientation_num) + " does not exist");
    }
  }
}

/**
 * @brief sets the status of the orientation input
 *
 * @param[in] status            - containing information on the status of the first orientation input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 * @param[in] orientation_num   - uint8_t:
 *                                containing the number of the orientation input [0-N]
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_orientation_status(
  const tam::types::ErrorLvl & status, uint8_t orientation_num)
{
  // forward the status of the orientation input to the state machine
  if (orientation_num < TConfig::NUM_ORIENTATION_MEASUREMENT) {
    state_machine_->set_orientation_status(status, orientation_num);
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: Orientation input " + std::to_string(orientation_num) + " does not exist");
  }
}

/**
 * @brief sets the status of the orientation input
 *
 * @param[in] status            - containing information on the status of the orientation input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 * @param[in] orientation_num   - uint8_t:
 *                                containing the number of the orientation input [0-N]
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_orientation_status(
  uint8_t status, uint8_t orientation_num)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  set_input_orientation_status(tam::type_conversions::error_type_from_diagnostic_level(
    static_cast<unsigned char>(status)), orientation_num);
}

/**
 * @brief forwards a detected timeout in the orientation input to the state estimation state machine
 *
 * @param[in] orientation_num   - uint8_t:
 *                                containing the number of the orientation input [0-N]
 */
template <typename TConfig> void StateEstimation<TConfig>::set_orientation_timeout(
  uint8_t orientation_num)
{
  // set the orientation input invalid in the state machine
  if (orientation_num < TConfig::NUM_ORIENTATION_MEASUREMENT) {
    state_machine_->set_orientation_invalid(orientation_num);
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: Orientation input " + std::to_string(orientation_num) + " does not exist");
  }
}

/**
 * @brief sets the linear velocity input
 *
 * @param[in] odometry          - tam::types::control::Odometry:
 *                                containing the linear velocity measured 
 * @param[in] vel_num           - uint8_t:
 *                                containing the number of the imu input [0-N]
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_linear_velocity(
  const tam::types::control::Odometry & odometry, uint8_t vel_num)
{
  if constexpr (!stateestimation::HasStateThetaRad<TConfig>) {
    // sets the linear velocity input in 2D
    // set the measurment vector
    if (vel_num < TConfig::NUM_VEL_MEASUREMENT) {
      // transform the 3D linear velocities into the footprint frame
      tam::types::common::Vector3D<double> odometry_footprint;
      odometry_footprint = tam::helpers::euler_rotations::vector_to_2d(odometry.velocity_mps,
                                                                      road_angles_.y,
                                                                      road_angles_.x);
      // set vx_VelCoG_mps
      z_[TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
        + vel_num * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
        + TConfig::MEASUREMENT_VX_MPS] = odometry_footprint.x;
      // set vy_VelCoG_mps
      z_[TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
        + vel_num * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
        + TConfig::MEASUREMENT_VY_MPS] = odometry_footprint.y;

      // set the bits corresponding to this linear velocity input in the fusion_vec to one
      // to fuse them iff the localization is also valid in the state machine
      fusion_vec_.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                          + vel_num * TConfig::VEL_MEASUREMENT_VECTOR_SIZE,
                          TConfig::VEL_MEASUREMENT_VECTOR_SIZE)
        = Eigen::VectorXd::Ones(TConfig::VEL_MEASUREMENT_VECTOR_SIZE);
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: Linear velocity input " + std::to_string(vel_num) + " does not exist");
    }
  } else {
    // set the linear velocity input in 3D
    // set the measurment vector
    if (vel_num < TConfig::NUM_VEL_MEASUREMENT) {
      // set vx_VelCoG_mps
      z_[TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + vel_num * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
        + TConfig::MEASUREMENT_VX_MPS] = odometry.velocity_mps.x;
      // set vy_VelCoG_mps
      z_[TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + vel_num * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
        + TConfig::MEASUREMENT_VY_MPS] = odometry.velocity_mps.y;
      // set vz_VelCoG_mps
      z_[TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + vel_num * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
        + TConfig::MEASUREMENT_VZ_MPS] = odometry.velocity_mps.z;

      // completly reject measurements that violate the hard treshold
      Eigen::Vector<double, TConfig::VEL_MEASUREMENT_VECTOR_SIZE> outlier_distance_se
        = tam::core::state::outlier_detection::squared_distance(
            x_out_.segment(TConfig::STATE_VX_MPS, TConfig::VEL_MEASUREMENT_VECTOR_SIZE),
            odometry.velocity_mps);
      Eigen::Vector<double, TConfig::VEL_MEASUREMENT_VECTOR_SIZE> outlier_distance_veh_model
        = tam::core::state::outlier_detection::squared_distance(
            vehicle_model_velocity_, odometry.velocity_mps);
      double outlier_threshold =
        param_manager_composer_->get_parameter_value("P_VDC_HardLinearVelocityOutlierTH").as_double();
      if ((outlier_distance_se.array() < outlier_threshold).all() || (0.0 == x_out_.segment(
          TConfig::STATE_VX_MPS, TConfig::VEL_MEASUREMENT_VECTOR_SIZE).array()).all()) {
        // set the bits corresponding to this linear velocity input in the fusion_vec to one
        // to fuse them iff the localization is also valid in the state machine
        fusion_vec_.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                            + vel_num * TConfig::VEL_MEASUREMENT_VECTOR_SIZE,
                            TConfig::VEL_MEASUREMENT_VECTOR_SIZE)
          = Eigen::VectorXd::Ones(TConfig::VEL_MEASUREMENT_VECTOR_SIZE);
        outlier_buffer_[vel_num] = 0;
      } else {
        outlier_buffer_[vel_num]++;
      }
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: Linear velocity input " + std::to_string(vel_num) + " does not exist");
    }
  }
}

/**
 * @brief sets the status of the linear velocity input
 *
 * @param[in] status            - containing information on the status of the linear velocity input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 * @param[in] vel_num           - uint8_t:
 *                                containing the number of the velocity input [0-N]
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_linear_velocity_status(
  const tam::types::ErrorLvl & status, uint8_t vel_num)
{
  // forward the status of the linear velocity input to the state machine
  if (vel_num < TConfig::NUM_VEL_MEASUREMENT) {
    int max_outlier_threshold =
      param_manager_composer_->get_parameter_value("P_VDC_MaxConsecutiveVelHardOutliers").as_int();
    if (outlier_buffer_[vel_num] < max_outlier_threshold) {
      state_machine_->set_linear_velocity_status(status, vel_num);
    } else {
      state_machine_->set_linear_velocity_status(tam::types::ErrorLvl::ERROR, vel_num);
    }
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: Linear velocity input " + std::to_string(vel_num) + " does not exist");
  }
}

/**
 * @brief sets the status of the linear velocity input
 *
 * @param[in] status            - containing information on the status of the linear velocity input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 * @param[in] vel_num           - uint8_t:
 *                                containing the number of the velocity input [0-N]
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_linear_velocity_status(
  uint8_t status, uint8_t vel_num)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  set_input_linear_velocity_status(tam::type_conversions::error_type_from_diagnostic_level(
    static_cast<unsigned char>(status)), vel_num);
}

/**
 * @brief forwards a detected timeout in the linear velocity input to the state estimation state machine
 *
 * @param[in] vel_num           - uint8_t:
 *                                containing the number of the velocity input [0-N]
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_linear_velocity_timeout(uint8_t vel_num)
{
  // set the linear velocity input invalid in the state machine
  if (vel_num < TConfig::NUM_VEL_MEASUREMENT) {
    state_machine_->set_linear_velocity_invalid(vel_num);
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: Linear velocity input " + std::to_string(vel_num) + " does not exist");
  }
}

/**
 * @brief sets the linear acceleration input
 *
 * @param[in] acceleration      - tam::types::control::AccelerationwithCovariances:
 *                                containing the acceleration of a IMU
 * @param[in] imu_num           - uint8_t:
 *                                containing the number of the imu input [0-N]
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_acceleration(
  const tam::types::control::AccelerationwithCovariances & acceleration, uint8_t imu_num)
{
  if constexpr (!stateestimation::HasStateThetaRad<TConfig>) {
    // set the linear acceleration input in 2D
    // set the raw input vector
    if (imu_num < (TConfig::NUM_IMU_MEASUREMENT
        + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
      // set ax_mps2 of the raw input vector
      u_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE
            + TConfig::INPUT_AX_MPS2] = acceleration.acceleration_mps2.x;
      // set ay_mps2 of the raw input vector
      u_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE
            + TConfig::INPUT_AY_MPS2] = acceleration.acceleration_mps2.y;

      // preprocess the input vector containing linear accelerations and angular veocities
      // the valid_u_fusion_vec ensures that only valid measurements are fused
      if (updated_angular_velocity_[imu_num] == 1) {
        u_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE,
          TConfig::INPUT_VECTOR_SIZE) =
            imu_handler_->filter_imu_measurements(
              u_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE,
                TConfig::INPUT_VECTOR_SIZE), imu_num);
        updated_angular_velocity_[imu_num] = 0;
      } else {
        updated_acceleration_[imu_num] = 1;
      }
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
    }
  } else {
    // set the linear acceleration input in 3D
    // set the raw input vector
    if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
      // completly reject measurements that violate the hard treshold
      Eigen::Vector<double,  3> outlier_distance
        = tam::core::state::outlier_detection::squared_distance(
            u_.segment(TConfig::INPUT_AX_MPS2, 3), acceleration.acceleration_mps2);
      if ((param_manager_composer_->get_parameter_value("P_VDC_HardAccelerometerOutlierTH").as_double()
          > outlier_distance.array()).all() || imu_num >= TConfig::NUM_IMU_MEASUREMENT || (0.0 ==
          u_.segment(TConfig::INPUT_AX_MPS2, 3).array()).all()) {
        // set ax_mps2 of the raw input vector
        u_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_AX_MPS2]
          = acceleration.acceleration_mps2.x;
        // set ay_mps2 of the raw input vector
        u_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_AY_MPS2]
          = acceleration.acceleration_mps2.y;
        // set az_mps2 of the raw input vector
        u_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_AZ_MPS2]
          = acceleration.acceleration_mps2.z;
        outlier_buffer_[TConfig::NUM_VEL_MEASUREMENT + TConfig::NUM_IMU_MEASUREMENT + imu_num] = 0;
      } else {
        outlier_buffer_[TConfig::NUM_VEL_MEASUREMENT + TConfig::NUM_IMU_MEASUREMENT + imu_num]++;
      }
      // preprocess the input vector containing linear accelerations and angular veocities
      // the valid_u_fusion_vec ensures that only valid measurements are fused
      if (updated_angular_velocity_[imu_num] == 1) {
        u_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE, TConfig::INPUT_VECTOR_SIZE) =
          imu_handler_->filter_imu_measurements(
            u_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE,
              TConfig::INPUT_VECTOR_SIZE), imu_num);
        updated_angular_velocity_[imu_num] = 0;
      } else {
        updated_acceleration_[imu_num] = 1;
      }
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
    }
  } 
}

/**
 * @brief sets the angular velocity input
 *
 * @param[in] odometry          - tam::types::control::Odometry:
 *                                containing the angular velocity of a IMU
 * @param[in] imu_num           - uint8_t:
 *                                containing the number of the imu input [0-N]
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_angular_velocity(
  const tam::types::control::Odometry & odometry, uint8_t imu_num)
{
  if constexpr (!stateestimation::HasStateThetaRad<TConfig>) {
    // set the angular velocity input in 2D
    // set the raw input vector
    if (imu_num < (TConfig::NUM_IMU_MEASUREMENT
        + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
      // set dpsi_rads of the raw input vector
      u_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE
            + TConfig::INPUT_DPSI_RADPS] = odometry.angular_velocity_radps.z;

      // preprocess the input vector containing linear accelerations and angular veocities
      // the valid_u_fusion_vec ensures that only valid measurements are fused
      if (updated_acceleration_[imu_num] == 1) {
        u_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE,
          TConfig::INPUT_VECTOR_SIZE) =
            imu_handler_->filter_imu_measurements(
              u_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE,
                TConfig::INPUT_VECTOR_SIZE), imu_num);
        updated_acceleration_[imu_num] = 0;
      } else {
        updated_angular_velocity_[imu_num] = 1;
      }
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
    }
  } else {
    // set the angular velocity input in 3D
    // set the raw input vector
    if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
      // completly reject measurements that violate the hard treshold
      Eigen::Vector<double,  3> outlier_distance
        = tam::core::state::outlier_detection::squared_distance(
            u_.segment(TConfig::INPUT_DPHI_RADPS, 3), odometry.angular_velocity_radps);
      if ((param_manager_composer_->get_parameter_value("P_VDC_HardAngularVelocityOutlierTH").as_double()
          > outlier_distance.array()).all() || imu_num >= TConfig::NUM_IMU_MEASUREMENT || (0.0 ==
          u_.segment(TConfig::INPUT_DPHI_RADPS, 3).array()).all()) {
        // set dphi_rads of the raw input vector
        u_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_DPHI_RADPS]
          = odometry.angular_velocity_radps.x;
        // set dtheta_rads of the raw input vector
        u_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_DTHETA_RADPS]
          = odometry.angular_velocity_radps.y;
        // set dpsi_rads of the raw input vector
        u_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_DPSI_RADPS]
          = odometry.angular_velocity_radps.z;
        outlier_buffer_[TConfig::NUM_VEL_MEASUREMENT + imu_num] = 0;
      } else {
        outlier_buffer_[TConfig::NUM_VEL_MEASUREMENT + imu_num]++;
      }
      // preprocess the input vector containing linear accelerations and angular veocities
      // the valid_u_fusion_vec ensures that only valid measurements are fused
      if (updated_acceleration_[imu_num] == 1) {
        u_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE, TConfig::INPUT_VECTOR_SIZE) =
          imu_handler_->filter_imu_measurements(
            u_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE,
              TConfig::INPUT_VECTOR_SIZE), imu_num);
        updated_acceleration_[imu_num] = 0;
      } else {
        updated_angular_velocity_[imu_num] = 1;
      }
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
    }
  }
}

/**
 * @brief sets the status of the imu input
 *
 * @param[in] status            - containing information on the status of the imu input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 * @param[in] imu_num           - uint8_t:
 *                                containing the number of the imu input [0-N]
 */
template <typename TConfig> void StateEstimation<TConfig>::set_input_imu_status(
  const tam::types::ErrorLvl & status, uint8_t imu_num)
{
  int max_outlier_threshold =
    param_manager_composer_->get_parameter_value("P_VDC_MaxConsecutiveIMUHardOutliers").as_int();
  tam::types::ErrorLvl imu_status = status;

  // outlier rejection
  if (imu_num < TConfig::NUM_IMU_MEASUREMENT) {
    if (outlier_buffer_[TConfig::NUM_VEL_MEASUREMENT + TConfig::NUM_IMU_MEASUREMENT + imu_num] >
          max_outlier_threshold ||
          outlier_buffer_[TConfig::NUM_VEL_MEASUREMENT + imu_num] > max_outlier_threshold) {
        imu_status = tam::types::ErrorLvl::ERROR;
    }
  }

  if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
    state_machine_->set_imu_status(imu_status, imu_num);
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
  }
}

/**
 * @brief sets the status of the imu input
 *
 * @param[in] status            - containing information on the status of the imu input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 * @param[in] imu_num           - uint8_t:
 *                                containing the number of the imu input [0-N]
 */
template <typename TConfig> void StateEstimation<TConfig>::set_input_imu_status(
  uint8_t status, uint8_t imu_num)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  set_input_imu_status(tam::type_conversions::error_type_from_diagnostic_level(
    static_cast<unsigned char>(status)), imu_num);
}

/**
 * @brief forwards a detected timeout in the imu input to the state estimation state machine
 *
 * @param[in] imu_num           - uint8_t:
 *                                containing the number of the localization input [0-N]
 */
template <typename TConfig> void StateEstimation<TConfig>::set_imu_timeout(
  uint8_t imu_num)
{
  // set the imu input invalid in the state machine
  if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
    state_machine_->set_imu_invalid(imu_num);
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
  }
}

/**
 * @brief sets the angular velocity of the wheels as input to convert them into linear velocities
 *
 * @param[in] wheel             - tam::types::common::DataPerWheel<double>:
 *                                angular velocity per wheel 
 */
template <typename TConfig> void StateEstimation<TConfig>::set_input_wheelspeeds(
  const tam::types::common::DataPerWheel<double> & wheel)
{
  // update the wheel speed odometry
  vehicle_model_handler_->input_wheel_angular_velocities(wheel);

  // set the linear velocity in the measurement vector (last measurement)
  set_input_linear_velocity(vehicle_model_handler_->get_vehicle_model_odometry(),
                            TConfig::NUM_VEL_MEASUREMENT - 1);

  // buffer the vehicle model output for debuging
  vehicle_model_velocity_ = vehicle_model_handler_->get_vehicle_model_odometry().velocity_mps;
}

/**
 * @brief sets the status of the wheelspeed input
 *
 * @param[in] status            - containing information on the status of the wheelspeed input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_wheelspeed_status(
  const tam::types::ErrorLvl & status)
{
  // update the status of the vehicle model handler
  vehicle_model_handler_->input_wheelspeed_status(status);

  // forward the updated status to the state machine
  set_input_linear_velocity_status(vehicle_model_handler_->get_status(),
                                   TConfig::NUM_VEL_MEASUREMENT - 1);
}

/**
 * @brief sets the status of the wheelspeed input
 *
 * @param[in] status            - containing information on the status of the wheelspeed input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_wheelspeed_status(
  uint8_t status)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  set_input_wheelspeed_status(tam::type_conversions::error_type_from_diagnostic_level(
    static_cast<unsigned char>(status)));
}

/**
 * @brief forwards a detected timeout in the wheelspeed input to the state estimation state machine
 */
template <typename TConfig>
  void StateEstimation<TConfig>::set_wheelspeeds_timeout(
    void)
{
  // update the status of the vehicle model handler
  vehicle_model_handler_->input_wheelspeed_status(tam::types::ErrorLvl::ERROR);

  // set the the last velocity input invalid in the state machine
  state_machine_->set_linear_velocity_invalid(TConfig::NUM_VEL_MEASUREMENT - 1);
}

// Steering angle input
/**
 * @brief sets the measured steering angle to use it in the vehicle model
 *
 * @param[in] steering_angle    - double:
 *                                steering angle in rad
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_steering_angle(double steering_angle)
{
  // update the steering angle in the vehicle model
  vehicle_model_handler_->input_steering_angle(steering_angle);
}

/**
 * @brief sets the status of the steering angle input
 *
 * @param[in] status            - containing information on the status of the steering angle input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_steering_angle_status(
  const tam::types::ErrorLvl & status)
{
  // update the status of the vehicle model handler
  vehicle_model_handler_->input_steering_angle_status(status);

  // forward the updated status to the state machine
  set_input_linear_velocity_status(vehicle_model_handler_->get_status(),
                                   TConfig::NUM_VEL_MEASUREMENT - 1);
}

/**
 * @brief sets the status of the steering angle input
 *
 * @param[in] status            - containing information on the status of the steering angle input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_steering_angle_status(uint8_t status)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  set_input_steering_angle_status(tam::type_conversions::error_type_from_diagnostic_level(
    static_cast<unsigned char>(status)));
}

/**
 * @brief forwards a detected timeout in the steering angle input to the state estimation state machine
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_steering_angle_timeout(void)
{
  // update the status of the vehicle model handler
  vehicle_model_handler_->input_steering_angle_status(tam::types::ErrorLvl::ERROR);

  // set the the last velocity input invalid in the state machine
  // if the steering angle is used in the vehicle model handler
  if (vehicle_model_handler_->get_status() != tam::types::ErrorLvl::OK) {
    state_machine_->set_linear_velocity_invalid(TConfig::NUM_VEL_MEASUREMENT - 1);
  }
}

/**
 * @brief sets the road angle input to compensate the IMU measurements
 *
 * @param[in] road_angles        - tam::types::common::Vector3D<double>:
 *                                  vector containing road angles
 */
template <typename TConfig> void StateEstimation<TConfig>::set_input_road_angles(
  const tam::types::common::Vector3D<double> & road_angles)
{
  road_angles_ = road_angles;

  if constexpr (stateestimation::HasStateThetaRad<TConfig>) {
    // additionally fuse the road angles to improve the orientation prediction for 3D filters
    if (param_manager_composer_->get_parameter_value("P_VDC_FuseRoadAngles").as_bool()) {
      Eigen::Vector<double, 2> orientation;
      orientation << road_angles.x, road_angles.y;

      // set the orientation in the measurement vector (penultimate measurement)
      z_.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                + (TConfig::NUM_ORIENTATION_MEASUREMENT - 2)
                * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE, 2) = orientation;

      // set bits in the fusion vector to indicate that a new measurement was received
      fusion_vec_.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                          + (TConfig::NUM_ORIENTATION_MEASUREMENT - 2)
                          * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE, 2)
        = Eigen::VectorXd::Ones(2);
    }
  }
}

/**
 * @brief sets the status for the road angles obtained from the roadhandler
 *
 * @param[in] status            - containing information on the status of the road angle input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_road_angle_status(
  const tam::types::ErrorLvl & status)
{
  // forward the status of the penultimate orientation input to the state machine
  // if the road angles should be fused
  if (param_manager_composer_->get_parameter_value("P_VDC_FuseRoadAngles").as_bool()) {
    state_machine_->set_orientation_status(status, TConfig::NUM_ORIENTATION_MEASUREMENT - 2);
  }
}

/**
 * @brief sets the status for the road angles obtained from the roadhandler
 *
 * @param[in] status            - containing information on the status of the road angle input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 */
template <typename TConfig>
void StateEstimation<TConfig>::set_input_road_angle_status(
  uint8_t status)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  set_input_road_angle_status(tam::type_conversions::error_type_from_diagnostic_level(
    static_cast<unsigned char>(status)));
}

/**
 * @brief sets the all calculated offsets in the imu_handler (based on the map data)
 */
template <typename TConfig> void StateEstimation<TConfig>::set_imu_offsets(void)
{
  if constexpr (!stateestimation::HasStateThetaRad<TConfig>) {
    // imu offsets in 2D
    Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>
      imu_bias = Eigen::Map<Eigen::VectorXd>(
        param_manager_composer_->get_parameter_value("P_VDC_InitialBias").as_double_array().data(),
        param_manager_composer_->get_parameter_value("P_VDC_InitialBias").as_double_array().size());

    // set the sensor biases for the imu handler
    imu_handler_->set_road_angles(road_angles_);
    imu_handler_->set_sensor_bias(imu_bias);
  } else {
    // imu offsets in 3D
    // ToDo(Marcel): Remove if we predict the bias or generate a function in kalman base class
    Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> imu_bias = Eigen::Map<Eigen::VectorXd>(
      param_manager_composer_->get_parameter_value("P_VDC_InitialBias").as_double_array().data(),
      param_manager_composer_->get_parameter_value("P_VDC_InitialBias").as_double_array().size());

    // set the sensor biases for the imu handler
    imu_handler_->set_sensor_bias(imu_bias);
  }
  
}

/**
 * @brief returns a pointer to the param manager composer
 *
 * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
 */
template <typename TConfig> std::shared_ptr<tam::interfaces::ParamManagerBase>
  StateEstimation<TConfig>::get_param_handler()
{
  return param_manager_composer_;
}

/**
 * @brief returns the odometry output predicted by the state estimation
 *
 * @param[out]                  - tam::types::control::Odometry
 */
template <typename TConfig> tam::types::control::Odometry
  StateEstimation<TConfig>::get_odometry(void)
{
  // map state estimation output to odometry type
  tam::types::control::Odometry output;
  if constexpr (!stateestimation::HasStateThetaRad<TConfig>) {
    // odometry output of 2D filters

    // get the covariance matrix of the kalman filter
    Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE,
                  TConfig::STATE_VECTOR_SIZE> P
      = kalman_filter_->get_covariance_matrix();

    // x,y,z coordintates
    output.position_m.x = x_out_[TConfig::STATE_POS_X_M];
    output.position_m.y = x_out_[TConfig::STATE_POS_Y_M];

    // roll, pitch, yaw
    output.orientation_rad.x = 0.0;
    output.orientation_rad.y = 0.0;
    output.orientation_rad.z = x_out_[TConfig::STATE_PSI_RAD];

    // linear velocity
    output.velocity_mps.x = x_out_[TConfig::STATE_VX_MPS];
    output.velocity_mps.y = x_out_[TConfig::STATE_VY_MPS];

    // angluar velocity
    output.angular_velocity_radps.z = u_[TConfig::INPUT_DPSI_RADPS];

    // set the covariance matrix for the position
    output.pose_covariance[0] = P(0, 0);
    output.pose_covariance[1] = P(0, 1);
    output.pose_covariance[6] = P(1, 0);
    output.pose_covariance[7] = P(1, 1);

    // set the covariance matrix for the orientation
    output.pose_covariance[35] = P(2, 2);

    // set the covariance matrix for the linear velocity
    output.velocity_covariance[0] = P(3, 3);
    output.velocity_covariance[1] = P(3, 4);
    output.velocity_covariance[6] = P(4, 3);
    output.velocity_covariance[7] = P(4, 4);
  } else {
    // odometry output of 3D filters
    // get the covariance matrix of the kalman filter
    Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> P
      = kalman_filter_->get_covariance_matrix();

    // x,y,z coordintates
    output.position_m.x = x_out_[TConfig::STATE_POS_X_M];
    output.position_m.y = x_out_[TConfig::STATE_POS_Y_M];
    output.position_m.z = x_out_[TConfig::STATE_POS_Z_M];

    // roll, pitch, yaw
    output.orientation_rad.x = x_out_[TConfig::STATE_PHI_RAD];
    output.orientation_rad.y = x_out_[TConfig::STATE_THETA_RAD];
    output.orientation_rad.z = x_out_[TConfig::STATE_PSI_RAD];

    // covariance matrix for position and orientation
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        output.pose_covariance[i * 6 + j] = P(i, j);
      }
    }

    // linear velocity
    output.velocity_mps.x = x_out_[TConfig::STATE_VX_MPS];
    output.velocity_mps.y = x_out_[TConfig::STATE_VY_MPS];
    output.velocity_mps.z = x_out_[TConfig::STATE_VZ_MPS];

    // angluar velocity
    output.angular_velocity_radps.x = u_[TConfig::INPUT_DPHI_RADPS];
    output.angular_velocity_radps.y = u_[TConfig::INPUT_DTHETA_RADPS];
    output.angular_velocity_radps.z = u_[TConfig::INPUT_DPSI_RADPS];

    // linear velocity covariance
    for (int i = 6; i < 9; ++i) {
      for (int j = 6; j < 9; ++j) {
        output.velocity_covariance[(i - 6) * 6 + j - 6] = P(i, j);
      }
    }
  }
  return output;
}

/**
 * @brief returns the linear acceleration output predicted by the state estimation
 *
 * @param[out]                  - tam::types::control::AccelerationwithCovariances
 */
template <typename TConfig> tam::types::control::AccelerationwithCovariances
  StateEstimation<TConfig>::get_acceleration(void)
{
  // map state estimation output to odometry type
  tam::types::control::AccelerationwithCovariances output;

  // linear accelerations in x and y
  output.acceleration_mps2.x = u_[TConfig::INPUT_AX_MPS2];
  output.acceleration_mps2.y = u_[TConfig::INPUT_AY_MPS2];

  if constexpr (stateestimation::HasStateThetaRad<TConfig>) {
    // additionally output the vertical acceleration for 3D filters
    output.acceleration_mps2.z = u_[TConfig::INPUT_AZ_MPS2];
  }

  return output;
}

/**
 * @brief returns the state estimation status
 *
 * @param[out]                  - tam::types::ErrorLvl
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 */
template <typename TConfig>
tam::types::ErrorLvl StateEstimation<TConfig>::get_status(void)
{
  return state_machine_->get_state();
}

/**
 * @brief returns the state estimation status
 *
 * @param[out]                  - uint8_t:
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 */
template <typename TConfig> uint8_t StateEstimation<TConfig>::get_status_as_uint8_t(
  void)
{
  return static_cast<uint8_t>(tam::type_conversions::diagnostic_level_from_type(
    state_machine_->get_state()));
}

/**
 * @brief returns the sideslip angle (Schwimmwinkel) predicted by the state estimation
 *
 * @param[out]                  - double
 */
template <typename TConfig>
double StateEstimation<TConfig>::get_sideslip_angle(void)
{
  // calculate the sideslip angle based on vx and vy in the vehicle coordinate frame
  if (x_out_[TConfig::STATE_VX_MPS] > 3.0) {
    return std::atan2(x_out_[TConfig::STATE_VY_MPS],
                      x_out_[TConfig::STATE_VX_MPS]);
  } else {
    return 0.0;
  }
}

/**
 * @brief returns a string containing the status message generated by the state machine
 *
 * @param[out]                  - std::string
 */
template <typename TConfig>
std::string StateEstimation<TConfig>::get_state_machine_status_msg(void)
{
  // forward the status message of the state machine
  return state_machine_->get_status_msg();
}

/**
 * @brief returns a debug container containing the state machine debug values
 *
 * @param[out]                  - tam::types::common::TUMDebugContainer::SharedPtr
 */
template <typename TConfig> tam::types::common::TUMDebugContainer::SharedPtr
  StateEstimation<TConfig>::get_state_machine_debug_output(void)
{
  // get the state machine debug output containing the valid bits
  std::map<std::string, bool> state_machine_debug = state_machine_->get_debug();

  // Iterate through the map
  for (std::map<std::string, bool>::const_iterator it = state_machine_debug.begin();
       it != state_machine_debug.end(); ++it) {
    // copy the debug map into the debug_container
    state_machine_debug_container_->log(it->first, it->second);
  }

  state_machine_debug_container_->log("overall_state", static_cast<uint8_t>(
    tam::type_conversions::diagnostic_level_from_type(state_machine_->get_state())));

  return state_machine_debug_container_;
}

/**
 * @brief returns a debug container containing the kalman filter debug values
 *
 * @param[out]                  - tam::types::common::TUMDebugContainer::SharedPtr
 */
template <typename TConfig> tam::types::common::TUMDebugContainer::SharedPtr
  StateEstimation<TConfig>::get_kalman_filter_debug_output(void)
{
  // get the state machine debug output containing the valid bits
  std::map<std::string, double> kalman_filter_debug = kalman_filter_->get_debug();

  // Iterate through the map
  for (std::map<std::string, double>::const_iterator it = kalman_filter_debug.begin();
       it != kalman_filter_debug.end(); ++it) {
    // copy the debug map into the debug_container
    kalman_filter_debug_container_->log(it->first, it->second);
  }

  if constexpr (stateestimation::HasStateThetaRad<TConfig>) {
    // additional logging for 3D filters
    kalman_filter_debug_container_->log("phi_road", road_angles_.x);
    kalman_filter_debug_container_->log("theta_road", road_angles_.y);

    kalman_filter_debug_container_->log("phi_ref", ref_angles_.x);
    kalman_filter_debug_container_->log("theta_ref", ref_angles_.y);

    kalman_filter_debug_container_->log("veh_m_vx_mps", vehicle_model_velocity_.x);
    kalman_filter_debug_container_->log("veh_m_vy_mps", vehicle_model_velocity_.y);

    // log the 3D states in the vehicle coordinate frame
    kalman_filter_debug_container_->log("veh_dphi_rads", u_[TConfig::INPUT_DPHI_RADPS]);
    kalman_filter_debug_container_->log("veh_dtheta_rads", u_[TConfig::INPUT_DTHETA_RADPS]);
    kalman_filter_debug_container_->log("veh_dpsi_rads", u_[TConfig::INPUT_DPSI_RADPS]);

    kalman_filter_debug_container_->log("veh_ax_mps2", u_[TConfig::INPUT_AX_MPS2]);
    kalman_filter_debug_container_->log("veh_ay_mps2", u_[TConfig::INPUT_AY_MPS2]);
    kalman_filter_debug_container_->log("veh_az_mps2", u_[TConfig::INPUT_AZ_MPS2]);
  }
  return kalman_filter_debug_container_;
}
}  // namespace tam::core::state
