// Copyright 2024 Sven Goblirsch
#pragma once
#include "ssa_estimation_cpp/ssa_estimation.hpp"
namespace tam::core::ssa
{
template <class TConfig>
SSAEstimation<TConfig>::SSAEstimation()
{
  // initialize all subclasses
  state_machine_ = std::make_unique<tam::core::ssa::StateMachine<TConfig>>();
  imu_handler_ = std::make_unique<tam::core::state::IMUHandler<TConfig>>();
  kalman_filter_ = std::make_unique<tam::core::ssa::UKF<TConfig>>();

  // param_manager
  param_manager_composer_ = std::make_shared<tam::core::ParamManagerComposer>(
    std::vector<std::shared_ptr<tam::interfaces::ParamManagerBase>>{
      state_machine_->get_param_handler(), imu_handler_->get_param_handler(),
      kalman_filter_->get_param_handler()});

  // Squared distance threshold on acceleration input used to set the IMU input invalid
  param_manager_composer_->declare_parameter(
    "P_VDC_HardAccelerometerOutlierTH", 1000.0, tam::types::param::ParameterType::DOUBLE, "");

  // Squared distance threshold on angular velocity input used to set the IMU input invalid
  param_manager_composer_->declare_parameter(
    "P_VDC_HardAngularVelocityOutlierTH", 1.0, tam::types::param::ParameterType::DOUBLE, "");

  // Number of Consecutive hard outliers before changing the State Machine status
  param_manager_composer_->declare_parameter(
    "P_VDC_MaxConsecutiveIMUHardOutliers", 50, tam::types::param::ParameterType::INTEGER, "");

  // allows overwrite the acceleration in z (vehicle coordinate frame)
  // with the vertical acceleration from the state estimation
  // Doing this can be necessary due the noise a_z imu measurements
  param_manager_composer_->declare_parameter(
    "P_VDC_Overwrite_Acceleration_Z", false, tam::types::param::ParameterType::BOOL, "");
  param_manager_composer_->declare_parameter(
    "P_VDC_InitialBias", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  // post filter for yaw_rate filtering to get a smooth yaw acceleartion signal
  param_manager_composer_->declare_parameter(
    "P_VDC_yaw_rate_post_filter_coefficients",
    std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");

  // set the state, input and measurent vector to zero
  x_out_.setZero();
  u_imu_.setZero();
  u_imu_raw_.setZero();
  z_meas_.setZero();
  outlier_buffer_.setZero();
  updated_acceleration_.setZero();
  updated_angular_velocity_.setZero();
}

template <class TConfig> void SSAEstimation<TConfig>::step()
{
  // set the sensor biases for the imu handler
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> imu_bias = Eigen::Map<Eigen::VectorXd>(
    param_manager_composer_->get_parameter_value("P_VDC_InitialBias").as_double_array().data(),
    param_manager_composer_->get_parameter_value("P_VDC_InitialBias").as_double_array().size());
  imu_handler_->set_sensor_bias(imu_bias);

  // preprocess the imu input vector containing linear accelerations and angular veocities
  // the valid_u_fusion_vec ensures that only valid measurements are fused
  u_imu_ = imu_handler_->update_input_vector(u_imu_raw_, state_machine_->get_valid_u_fusion_vec());

  // additionally filter yaw rate to use a smooth version for the derivate
  if (initialize_filter_) {
    // set the Filter Coefficients
    Eigen::VectorXd filter_coefficients = Eigen::Map<Eigen::VectorXd>(
        param_manager_composer_->get_parameter_value(
            "P_VDC_yaw_rate_post_filter_coefficients").as_double_array().data(),
        param_manager_composer_->get_parameter_value(
            "P_VDC_yaw_rate_post_filter_coefficients").as_double_array().size());
    filter_yaw_rate_radps_ = std::make_unique<tam::core::state::FIR>(filter_coefficients);
    initialize_filter_ = false;
  }
  double yaw_rate_radps = filter_yaw_rate_radps_->step(u_imu_[TConfig::INPUT_DPSI_RADPS]);
  // calculate yaw acceleration
  yaw_acc_radps2_ = (yaw_rate_radps - prev_yaw_rate_radps_) / TConfig::TS;
  prev_yaw_rate_radps_ = yaw_rate_radps;
  z_meas_[TConfig::MEASUREMENT_YAW_ACC_RADPS2] = yaw_acc_radps2_;
  // step the kalman filter
  // create input vectors for the kalman filter based on imu and remaining measurements
  Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE> u =
    kalman_filter_->get_u(u_imu_, z_meas_);
  Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE> z =
    kalman_filter_->get_z(u_imu_, z_meas_);
  // prediction step
  kalman_filter_->predict(u);
  // corection step
  kalman_filter_->update(z);
  x_out_ = kalman_filter_->get_state_vector();
  state_machine_->check_output_feasability(x_out_);
  state_machine_->update();
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_acceleration(
  const tam::types::control::AccelerationwithCovariances & acceleration, const uint8_t imu_num)
{
  // set the raw input vector
  if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
    // completly reject measurements that violate the hard treshold
    Eigen::Vector<double, 3> acc_vec;
    acc_vec << acceleration.acceleration_mps2.x, acceleration.acceleration_mps2.y,
      acceleration.acceleration_mps2.z;
    // outlier distance = measurement - filtered value
    Eigen::Vector<double,  3> outlier_distance;
    outlier_distance = acc_vec.array().abs()
      - u_imu_.segment(TConfig::INPUT_AX_MPS2, 3).array().abs();
    Eigen::Vector<double,  3> outlier_distance_sqr;
    outlier_distance_sqr = outlier_distance.cwiseProduct(outlier_distance);
    if (
      (param_manager_composer_->get_parameter_value("P_VDC_HardAccelerometerOutlierTH").as_double()
         > outlier_distance_sqr.array()).all() || imu_num >= TConfig::NUM_IMU_MEASUREMENT ||
         (0.0 == u_imu_.segment(TConfig::INPUT_AX_MPS2, 3).array()).all()) {
      // set ax_mps2 of the raw input vector
      u_imu_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_AX_MPS2]
        = acceleration.acceleration_mps2.x;
      // set ay_mps2 of the raw input vector
      u_imu_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_AY_MPS2]
        = acceleration.acceleration_mps2.y;
      // set az_mps2 of the raw input vector
      u_imu_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_AZ_MPS2]
        = acceleration.acceleration_mps2.z;
      outlier_buffer_[TConfig::NUM_IMU_MEASUREMENT + imu_num] = 0;
    } else {
      outlier_buffer_[TConfig::NUM_IMU_MEASUREMENT + imu_num]++;
    }
    // preprocess the input vector containing linear accelerations and angular veocities
    // the valid_u_fusion_vec ensures that only valid measurements are fused
    if (updated_angular_velocity_[imu_num] == 1) {
      u_imu_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE, 6) =
        imu_handler_->filter_imu_measurements(
          u_imu_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE, 6), imu_num);
      updated_angular_velocity_[imu_num] = 0;
    } else {
      updated_acceleration_[imu_num] = 1;
    }
  } else {
    throw std::invalid_argument(
      "[SSAEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
  }
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_angular_velocity(
  const tam::types::control::Odometry & odometry, const uint8_t imu_num)
{
  // set the raw input vector
  if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
    // completly reject measurements that violate the hard treshold
    Eigen::Vector<double, 3> angvel_vec;
    angvel_vec << odometry.angular_velocity_radps.x, odometry.angular_velocity_radps.y,
      odometry.angular_velocity_radps.z;
    // outlier distance = measurement - filtered value
    Eigen::Vector<double,  3> outlier_distance;
    outlier_distance = angvel_vec.array().abs()
      - u_imu_.segment(TConfig::INPUT_DPHI_RADPS, 3).array().abs();
    Eigen::Vector<double,  3> outlier_distance_sqr;
    outlier_distance_sqr = outlier_distance.cwiseProduct(outlier_distance);
    if (
      (param_manager_composer_->get_parameter_value("P_VDC_HardAngularVelocityOutlierTH").as_double()
         > outlier_distance_sqr.array()).all() || imu_num >= TConfig::NUM_IMU_MEASUREMENT || (0.0 ==
         u_imu_.segment(TConfig::INPUT_DPHI_RADPS, 3).array()).all()) {
      // set dphi_rads of the raw input vector
      u_imu_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_DPHI_RADPS]
        = odometry.angular_velocity_radps.x;
      // set dtheta_rads of the raw input vector
      u_imu_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_DTHETA_RADPS]
        = odometry.angular_velocity_radps.y;
      // set dpsi_rads of the raw input vector
      u_imu_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_DPSI_RADPS]
        = odometry.angular_velocity_radps.z;
      outlier_buffer_[imu_num] = 0;
    } else {
      outlier_buffer_[imu_num]++;
    }
    // preprocess the input vector containing linear accelerations and angular veocities
    // the valid_u_fusion_vec ensures that only valid measurements are fused
    if (updated_acceleration_[imu_num] == 1) {
      u_imu_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE, 6) =
        imu_handler_->filter_imu_measurements(
          u_imu_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE, 6), imu_num);
      updated_acceleration_[imu_num] = 0;
    } else {
      updated_angular_velocity_[imu_num] = 1;
    }
  } else {
    throw std::invalid_argument(
      "[SSAEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
  }
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_imu_status(
  const tam::types::ErrorLvl & status, const uint8_t imu_num)
{
  int max_outlier_threshold =
    param_manager_composer_->get_parameter_value("P_VDC_MaxConsecutiveIMUHardOutliers").as_int();
  tam::types::ErrorLvl imu_status = status;

  // outlier rejection
  if (imu_num < TConfig::NUM_IMU_MEASUREMENT) {
    if (outlier_buffer_[TConfig::NUM_IMU_MEASUREMENT + imu_num] > max_outlier_threshold ||
          outlier_buffer_[imu_num] > max_outlier_threshold) {
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

template <class TConfig> void SSAEstimation<TConfig>::set_input_imu_status(
  const uint8_t status, const uint8_t imu_num)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  set_input_imu_status(
    tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)),
    imu_num);
}

template <class TConfig> void SSAEstimation<TConfig>::set_imu_timeout(const uint8_t imu_num)
{
  // set the imu input invalid in the state machine
  if (imu_num < TConfig::NUM_IMU_MEASUREMENT) {
    state_machine_->set_imu_invalid(imu_num);
  } else {
    throw std::invalid_argument(
      "[SSAEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
  }
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_wheelspeeds(
  const tam::types::common::DataPerWheel<double> & wheel)
{
  z_meas_[TConfig::MEASUREMENT_OMEGA_WHEEL_FL_RADPS] = wheel.front_left;
  z_meas_[TConfig::MEASUREMENT_OMEGA_WHEEL_FR_RADPS] = wheel.front_right;
  z_meas_[TConfig::MEASUREMENT_OMEGA_WHEEL_RL_RADPS] = wheel.rear_left;
  z_meas_[TConfig::MEASUREMENT_OMEGA_WHEEL_RR_RADPS] = wheel.rear_right;
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_wheelspeed_status(
  const tam::types::ErrorLvl & status)
{
  // forward the updated status to the state machine
  state_machine_->set_wheelspeed_status(status);
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_wheelspeed_status(
  const uint8_t status)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  state_machine_->set_wheelspeed_status(tam::type_conversions::error_type_from_diagnostic_level(
    static_cast<unsigned char>(status)));
}

template <class TConfig> void SSAEstimation<TConfig>::set_wheelspeed_timeout()
{
  // set the last wheelspeed input invalid in the state machine
  state_machine_->set_wheelspeed_status(tam::types::ErrorLvl::ERROR);
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_steering_angle(
  const double steering_angle)
{
  z_meas_[TConfig::MEASUREMENT_DELTA_RAD] = steering_angle;
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_steering_angle_status(
  const tam::types::ErrorLvl & status)
{
  // forward the updated status to the state machine
  state_machine_->set_steering_status(status);
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_steering_angle_status(
  const uint8_t status)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  state_machine_->set_steering_status(
    tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)));
}

template <class TConfig> void SSAEstimation<TConfig>::set_steering_angle_timeout()
{
  // set the last steering input invalid in the state machine
  state_machine_->set_steering_status(tam::types::ErrorLvl::ERROR);
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_drivetrain_torque(
  const double drivetrain_trq_Nm)
{
  z_meas_[TConfig::MEASUREMENT_DRIVETRAIN_TORQUE_NM] = drivetrain_trq_Nm;
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_drivetrain_torque_status(
  const tam::types::ErrorLvl & status)
{
  // forward the updated status to the state machine
  state_machine_->set_drivetrain_torque_status(status);
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_drivetrain_torque_status(
  const uint8_t status)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  state_machine_->set_drivetrain_torque_status(
    tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)));
}

template <class TConfig> void SSAEstimation<TConfig>::set_drivetrain_torque_timeout()
{
  // set the last drivetrain torque input invalid in the state machine
  state_machine_->set_drivetrain_torque_status(tam::types::ErrorLvl::ERROR);
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_brake_pressure(
  const tam::types::common::DataPerWheel<double> & wheel)
{
  z_meas_[TConfig::MEASUREMENT_BRAKE_PRESSURE_FL_PA] = wheel.front_left;
  z_meas_[TConfig::MEASUREMENT_BRAKE_PRESSURE_FR_PA] = wheel.front_right;
  z_meas_[TConfig::MEASUREMENT_BRAKE_PRESSURE_RL_PA] = wheel.rear_left;
  z_meas_[TConfig::MEASUREMENT_BRAKE_PRESSURE_RR_PA] = wheel.rear_right;
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_brake_pressure_status(
  const tam::types::ErrorLvl & status)
{
  // forward the updated status to the state machine
  state_machine_->set_brake_pressure_status(status);
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_brake_pressure_status(
  const uint8_t status)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  state_machine_->set_brake_pressure_status(
    tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)));
}

template <class TConfig> void SSAEstimation<TConfig>::set_brake_pressure_timeout()
{
  // set the last brake pressure input invalid in the state machine
  state_machine_->set_brake_pressure_status(tam::types::ErrorLvl::ERROR);
}

template <class TConfig> void SSAEstimation<TConfig>::set_input_vehicle_orientation(
  const tam::types::control::Odometry & odometry)
{
  z_meas_[TConfig::MEASUREMENT_ROAD_ANGLES_ROLL_RAD] = odometry.orientation_rad.x;
  z_meas_[TConfig::MEASUREMENT_ROAD_ANGLES_PITCH_RAD] = odometry.orientation_rad.y;
}

template <class TConfig> void SSAEstimation<TConfig>:: set_input_orientation_status(
  const tam::types::ErrorLvl & status)
{
  // forward the updated status to the state machine
  state_machine_->set_orientation_status(status);
}

template <class TConfig> void SSAEstimation<TConfig>:: set_input_orientation_status(
  const uint8_t status)
{
  // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
  state_machine_->set_orientation_status(
    tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)));
}

template <class TConfig> void SSAEstimation<TConfig>:: set_orientation_timeout()
{
  // set the last orientation input invalid in the state machine
  state_machine_->set_orientation_status(tam::types::ErrorLvl::ERROR);
}

template <class TConfig> std::shared_ptr<tam::interfaces::ParamManagerBase> SSAEstimation<TConfig>::get_param_handler()
{
  return param_manager_composer_;
}

template <class TConfig> tam::types::control::Odometry SSAEstimation<TConfig>::get_odometry()
{
  // map ssa estimation output to odometry type
  tam::types::control::Odometry output;

  // linear velocity
  output.velocity_mps.x = x_out_[TConfig::STATE_VEL_MPS] *
    std::cos(x_out_[TConfig::STATE_BETA_RAD]);
  output.velocity_mps.y = x_out_[TConfig::STATE_VEL_MPS] *
    std::sin(x_out_[TConfig::STATE_BETA_RAD]);
  output.velocity_mps.z = 0.0;

  Eigen::Matrix<double, 2, 2> P = kalman_filter_->get_linear_velocity_covariance();
  // linear velocity covariance
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      output.velocity_covariance[i * 6 + j] = P(i, j);
    }
  }
  return output;
}

template <class TConfig> tam::types::ErrorLvl SSAEstimation<TConfig>::get_status()
{
  return state_machine_->get_state();
}

template <class TConfig> uint8_t SSAEstimation<TConfig>::get_status_as_uint8_t()
{
  return static_cast<uint8_t>(tam::type_conversions::diagnostic_level_from_type(
    state_machine_->get_state()));
}

template <class TConfig> std::string SSAEstimation<TConfig>::get_state_machine_status_msg()
{
  // forward the status message of the state machine
  return state_machine_->get_status_msg();
}

template <class TConfig> tam::types::common::TUMDebugContainer::SharedPtr
  SSAEstimation<TConfig>::get_state_machine_debug_output()
{
  // get the state machine debug output containing the valid bits
  std::map<std::string, bool> state_machine_debug = state_machine_->get_debug();

  // Iterate through the map
  for (std::map<std::string, bool>::const_iterator it = state_machine_debug.begin();
       it != state_machine_debug.end(); ++it) {
    // copy the debug map into the debug_container
    state_machine_debug_container_->log(it->first, it->second);
  }
  state_machine_debug_container_->log(
    "overall_state", static_cast<uint8_t>(tam::type_conversions::diagnostic_level_from_type(
                       state_machine_->get_state())));

  return state_machine_debug_container_;
}

template <class TConfig> tam::types::common::TUMDebugContainer::SharedPtr
  SSAEstimation<TConfig>::get_kalman_filter_debug_output()
{
  // get the state machine debug output containing the valid bits
  std::map<std::string, double> kalman_filter_debug = kalman_filter_->get_debug();

  // Iterate through the map
  for (std::map<std::string, double>::const_iterator it = kalman_filter_debug.begin();
       it != kalman_filter_debug.end(); ++it) {
    // copy the debug map into the debug_container
    kalman_filter_debug_container_->log(it->first, it->second);
  }
  // Used orientation angles
  kalman_filter_debug_container_->log("roll_angle_rad",
    z_meas_[TConfig::MEASUREMENT_ROAD_ANGLES_ROLL_RAD]);
  kalman_filter_debug_container_->log("pitch_angle_rad",
    z_meas_[TConfig::MEASUREMENT_ROAD_ANGLES_PITCH_RAD]);
  // Used acceleration signals
  kalman_filter_debug_container_->log("filtered_acc_x_mps2",
    u_imu_[TConfig::INPUT_AX_MPS2]);
  kalman_filter_debug_container_->log("filtered_acc_y_mps2",
    u_imu_[TConfig::INPUT_AY_MPS2]);
  kalman_filter_debug_container_->log("filtered_acc_z_mps2",
    u_imu_[TConfig::INPUT_AZ_MPS2]);
  kalman_filter_debug_container_->log("filtered_dpsi_radps",
    u_imu_[TConfig::INPUT_DPSI_RADPS]);
  kalman_filter_debug_container_->log("yaw_acc_radps2", yaw_acc_radps2_);
  return kalman_filter_debug_container_;
}
}  // namespace tam::core::ssa
