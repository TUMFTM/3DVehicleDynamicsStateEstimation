// Copyright 2024 Sven Goblirsch
#pragma once
#include "ssa_estimation_cpp/submodules/ukf.hpp"
namespace tam::core::ssa
{
template <class TConfig> UKF<TConfig>::UKF()
{
  // ensure that all states are set to zero
  x_.setZero();
  x_pred_.setZero();
  // initialize param_manager
  param_manager_ = std::make_shared<tam::core::ParamManager>();
  // declare vehicle parameters
  param_manager_->declare_parameter(
    "P_VDC_mass_kg", double{800.00}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_Izz_kgm2", double{1000.00}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_l_front_m", double{1.724}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_l_rear_m", double{1.247}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_tw_front_m", double{1.639}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_tw_rear_m", double{1.524}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_rho_air_kgpm3", double{1.22}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_cL_front", double{-0.65}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_cL_rear", double{-0.8}, tam::types::param::ParameterType::DOUBLE, "");
  // declare brake parameters
  param_manager_->declare_parameter(
    "P_VDC_d_brake_bore_front_m", double{0.0798}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_d_brake_bore_rear_m", double{0.0798}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_r_brake_pads_lever_front_m", double{0.1493}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_r_brake_pads_lever_rear_m", double{0.1493}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_mue_brakes_front", double{0.55}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_mue_brakes_rear", double{0.55}, tam::types::param::ParameterType::DOUBLE, "");
  // declare tire parameters
  param_manager_->declare_parameter(
    "P_VDC_rtire_front_m", double{0.2999}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_rtire_rear_m", double{0.3120}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_tire_MF_long_front", std::vector<double>{15.0, 1.40, 1.55, 1.0},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  param_manager_->declare_parameter(
    "P_VDC_tire_MF_long_rear", std::vector<double>{15.0, 1.40, 1.60, 1.00},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  param_manager_->declare_parameter(
    "P_VDC_tire_MF_lat_front", std::vector<double>{10.0, 1.30, 1.40, 1.0},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  param_manager_->declare_parameter(
    "P_VDC_tire_MF_lat_rear", std::vector<double>{15.0, 1.40, 1.65, 1.00},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  // declare parameters for sigma point distribution
  param_manager_->declare_parameter(
    "P_VDC_alpha", double{0.001}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_beta", double{2.0}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_kappa", double{0.0}, tam::types::param::ParameterType::DOUBLE, "");
  // declare parameters containing the initial covariance matrices
  param_manager_->declare_parameter(
    "P_VDC_P_Init",
    std::vector<double>{0.5, 0.0004}, tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  param_manager_->declare_parameter(
    "P_VDC_ProcessCov_Q",
    std::vector<double>{0.025, 0.00015}, tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  // R: v_stm, v_stm, FxTf, FxTr, FyTf, FyTr
  param_manager_->declare_parameter(
    "P_VDC_MeasCov_R",
    std::vector<double>{1.00, 1.00, 200.0, 200.0, 150.0, 150.0},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  // define auxilary variables
  initialize_ = true;
  vel_min_mps_ = 5.0;
}

template <class TConfig> void
  UKF<TConfig>::initialize()
{
  // Set initial state covariance
  P_.diagonal() = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_P_Init").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_P_Init").as_double_array().size());
  P_ = P_.array().square();
  // Set Kalman Filter Noise Matrices
  R_.diagonal() = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().size());
  R_ = R_.array().square();
  Q_.diagonal() = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_ProcessCov_Q").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_ProcessCov_Q").as_double_array().size());
  Q_ = Q_.array().square();
  // auxilary variables
  l_m_ = param_manager_->get_parameter_value("P_VDC_l_front_m").as_double() +
    param_manager_->get_parameter_value("P_VDC_l_rear_m").as_double();
  lambda_ = std::pow(param_manager_->get_parameter_value("P_VDC_alpha").as_double(), 2) *
    (x_.size() + param_manager_->get_parameter_value("P_VDC_kappa").as_double()) - x_.size();

  // Initialize Sigma Point Weights acc. to R. Van der Merwe
  const double c = 0.5 / (x_.size() + lambda_);
  W_mean_ = Eigen::VectorXd::Constant(2 * x_.size() + 1, c);
  W_cov_ = Eigen::VectorXd::Constant(2 * x_.size() + 1, c);
  W_mean_[0] = lambda_ / (x_.size() + lambda_);
  W_cov_[0] = W_mean_[0] +
    (1.0 - std::pow(param_manager_->get_parameter_value("P_VDC_alpha").as_double(), 2) +
    param_manager_->get_parameter_value("P_VDC_beta").as_double());
}

template <class TConfig> void
  UKF<TConfig>::predict(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE>> & u)
{
  // calculate velocity according to wheel speed average of the front wheels
  double vel_wheel_mps = (param_manager_->get_parameter_value("P_VDC_rtire_front_m").as_double() *
      (u[TConfig::PROCESS_OMEGA_WHEEL_FL_RADPS] + u[TConfig::PROCESS_OMEGA_WHEEL_FR_RADPS]) / 2);
  if (initialize_) {
    // initialize the filter
    tam::core::ssa::UKF<TConfig>::initialize();
    x_[0] = vel_wheel_mps;
    x_[1] = 0.0;
    initialize_ = false;
    init_time_s_ = 0.0;
  } else if (std::abs(vel_wheel_mps) <= 2.0) {
    // set the filter states to no side slip and wheel velocity
    // for velocities below 2 mps to avoid instabilities
    x_[0] = vel_wheel_mps;
    x_[1] = 0.0;
    P_.diagonal() = Eigen::Map<Eigen::VectorXd>(
        param_manager_->get_parameter_value("P_VDC_P_Init").as_double_array().data(),
        param_manager_->get_parameter_value("P_VDC_P_Init").as_double_array().size());
    P_ = P_.array().square();
    init_time_s_ = 0.0;
  } else {
    // Create Sigma Points acc. to R. Van der Merwe
    // Predict Covariance of sigma points using Cholesky Decomposition
    P_pred_ = ((x_.size() + lambda_) * P_).llt().matrixL();
    // Create Sigma Points
    Eigen::MatrixXd sigmaPoints(x_.size(), 2 * x_.size() + 1);
    sigmaPoints.col(0) = x_;
    for (int i = 0; i < x_.size(); ++i) {
        sigmaPoints.col(i + 1) = x_ - P_pred_.col(i);
        sigmaPoints.col(x_.size() + i + 1) = x_ + P_pred_.col(i);
    }
    // Calculate Process Sigmas
    for (int i = 0; i < sigmaPoints_pred_.cols(); ++i) {
        sigmaPoints_pred_.col(i) =
          tam::core::ssa::UKF<TConfig>::process_function(sigmaPoints.col(i), u);
    }
    // Unscented transform
    x_pred_.setZero();
    P_pred_.setZero();
    for (int i = 0; i < sigmaPoints_pred_.cols(); ++i) {
        x_pred_ += W_mean_[i] * sigmaPoints_pred_.col(i);
    }
    for (int i = 0; i < sigmaPoints_pred_.cols(); ++i) {
      Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> residual =
        sigmaPoints_pred_.col(i) - x_pred_;
      P_pred_ += W_cov_[i] * residual * residual.transpose();
    }
    // Add Process Noise Q
    Q_.diagonal() = Eigen::Map<Eigen::VectorXd>(
      param_manager_->get_parameter_value("P_VDC_ProcessCov_Q").as_double_array().data(),
      param_manager_->get_parameter_value("P_VDC_ProcessCov_Q").as_double_array().size());
    Q_ = Q_.array().square();
    P_pred_ += Q_;
  }
}

template <class TConfig> void
  UKF<TConfig>::update(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z)
{
  // y_sigmapoints = h(sigmapoints)
  Eigen::MatrixXd sigmaPoints_meas(z_virt_.size(), 2 * x_.size() + 1);
  for (int i = 0; i < sigmaPoints_meas.cols(); ++i) {
    sigmaPoints_meas.col(i) = tam::core::ssa::UKF<TConfig>::transfer_function(
      sigmaPoints_pred_.col(i), z);
  }
  // Unscented transform
  // y_mean = W_mean * y_sigmapoints
  // Cov = W_cov * y_sigma - y_mean * y_sigma - y_mean + R_cov
  // Cross_cov = W_cov * x_sigma - x_mean * y_sigma -y_mean
  y_.setZero();
  P_meas_.setZero();
  P_cross_.setZero();
  for (int i = 0; i < sigmaPoints_meas.cols(); ++i) {
    y_ += W_mean_[i] * sigmaPoints_meas.col(i);
  }
  for (int i = 0; i < sigmaPoints_meas.cols(); ++i) {
    P_meas_ += W_cov_[i] * (sigmaPoints_meas.col(i) - y_) *
        (sigmaPoints_meas.col(i) - y_).transpose();
    P_cross_ += W_cov_[i] * (sigmaPoints_pred_.col(i) - x_pred_) *
        (sigmaPoints_meas.col(i) - y_).transpose();
  }
  // Add virtual measurement Noise R
  tam::core::ssa::UKF<TConfig>::adaptR(z);
  P_meas_ += R_;
  // K_gain = Cross_cov * inv(Cov)
  K_Gain_ = P_cross_ * P_meas_.inverse();

  // get virtual measurement vector
  z_virt_ = tam::core::ssa::UKF<TConfig>::get_virtual_measurement(z);

  // limit state update in the first 20 filter steps (0.2 seconds for 100 Hz filter rate)
  // to enhance convergence
  if (init_time_s_ < 20.0) {
      x_[0] = std::clamp(x_[0], z_virt_[0] - 0.40, z_virt_[0] + 0.40);
      x_[1] = std::clamp(x_[1], -0.00001, 0.00001);
      ++init_time_s_;
  } else {
      x_meas_update_ = K_Gain_ * (z_virt_ - y_);
      x_ = x_pred_ + K_Gain_ * (z_virt_ - y_);
      P_ = P_pred_ - K_Gain_ * P_meas_ * K_Gain_.transpose();
      // limit state deviation from wheelspeed for low speed
      // to enable smooth transition from zero velocity
      if (z_virt_[0] <= vel_min_mps_) {
        x_[0] = std::clamp(x_[0], z_virt_[0] - 0.25, z_virt_[0] + 0.25);
        x_[1] = std::clamp(x_[1], -0.0001, 0.0001);
      }
  }
  // calculate tire states, loads and axle forces for the final state for logging
  // calculate tire slip
  Eigen::Matrix<double, 4, 2> slip_tire_perc =
    tam::core::ssa::UKF<TConfig>::calc_tire_states(x_, z);
  slip_tire_long_perc_ = slip_tire_perc.col(0);
  slip_tire_lat_perc_ = slip_tire_perc.col(1);
  // calculate vertical load of each tire
  load_tire_N_ =
    tam::core::ssa::UKF<TConfig>::calc_tire_loads(x_, z);
  // calculate axle forces considering the specified tire model
  Eigen::Vector<double, 6> axle_forces_N =
    tam::core::ssa::UKF<TConfig>::calc_axle_forces(
      slip_tire_long_perc_, slip_tire_lat_perc_, load_tire_N_);
  force_axle_front_N_ = axle_forces_N.segment(0, 3);
  force_axle_rear_N_ = axle_forces_N.segment(3, 3);
}

template <class TConfig> Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE>
  UKF<TConfig>::get_u(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u_imu,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & z_meas)
{
  Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE> u;
  u[TConfig::PROCESS_DELTA_RAD] = z_meas[TConfig::MEASUREMENT_DELTA_RAD];
  u[TConfig::PROCESS_OMEGA_WHEEL_FL_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_FL_RADPS];
  u[TConfig::PROCESS_OMEGA_WHEEL_FR_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_FR_RADPS];
  u[TConfig::PROCESS_OMEGA_WHEEL_RL_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_RL_RADPS];
  u[TConfig::PROCESS_OMEGA_WHEEL_RR_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_RR_RADPS];
  u[TConfig::PROCESS_DPSI_RADPS] = u_imu[TConfig::INPUT_DPSI_RADPS];
  u[TConfig::PROCESS_AX_MPS2] = u_imu[TConfig::INPUT_AX_MPS2];
  u[TConfig::PROCESS_AY_MPS2] = u_imu[TConfig::INPUT_AY_MPS2];
  u[TConfig::PROCESS_AZ_MPS2] = u_imu[TConfig::INPUT_AZ_MPS2];
  u[TConfig::PROCESS_ROAD_ANGLES_PITCH_RAD] = z_meas[TConfig::MEASUREMENT_ROAD_ANGLES_PITCH_RAD];
  u[TConfig::PROCESS_ROAD_ANGLES_ROLL_RAD] = z_meas[TConfig::MEASUREMENT_ROAD_ANGLES_ROLL_RAD];
  return u;
}

template <class TConfig> Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>
  UKF<TConfig>::get_z(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u_imu,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & z_meas)
{
  Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE> z;
  z[TConfig::UPDATE_DELTA_RAD] = z_meas[TConfig::MEASUREMENT_DELTA_RAD];
  z[TConfig::UPDATE_OMEGA_WHEEL_FL_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_FL_RADPS];
  z[TConfig::UPDATE_OMEGA_WHEEL_FR_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_FR_RADPS];
  z[TConfig::UPDATE_OMEGA_WHEEL_RL_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_RL_RADPS];
  z[TConfig::UPDATE_OMEGA_WHEEL_RR_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_RR_RADPS];
  z[TConfig::UPDATE_DPSI_RADPS] = u_imu[TConfig::INPUT_DPSI_RADPS];
  z[TConfig::UPDATE_AX_MPS2] = u_imu[TConfig::INPUT_AX_MPS2];
  z[TConfig::UPDATE_AY_MPS2] = u_imu[TConfig::INPUT_AY_MPS2];
  z[TConfig::UPDATE_AZ_MPS2] = u_imu[TConfig::INPUT_AZ_MPS2];
  z[TConfig::UPDATE_DRIVETRAIN_TORQUE_NM] = z_meas[TConfig::MEASUREMENT_DRIVETRAIN_TORQUE_NM];
  z[TConfig::UPDATE_BRAKE_PRESSURE_FL_PA] = z_meas[TConfig::MEASUREMENT_BRAKE_PRESSURE_FL_PA];
  z[TConfig::UPDATE_BRAKE_PRESSURE_FR_PA] = z_meas[TConfig::MEASUREMENT_BRAKE_PRESSURE_FR_PA];
  z[TConfig::UPDATE_BRAKE_PRESSURE_RL_PA] = z_meas[TConfig::MEASUREMENT_BRAKE_PRESSURE_RL_PA];
  z[TConfig::UPDATE_BRAKE_PRESSURE_RR_PA] = z_meas[TConfig::MEASUREMENT_BRAKE_PRESSURE_RR_PA];
  z[TConfig::UPDATE_YAW_ACC_RADPS2] = z_meas[TConfig::MEASUREMENT_YAW_ACC_RADPS2];
  z[TConfig::UPDATE_ROAD_ANGLES_PITCH_RAD] = z_meas[TConfig::MEASUREMENT_ROAD_ANGLES_PITCH_RAD];
  z[TConfig::UPDATE_ROAD_ANGLES_ROLL_RAD] = z_meas[TConfig::MEASUREMENT_ROAD_ANGLES_ROLL_RAD];
  return z;
}

template <class TConfig> void
  UKF<TConfig>::adaptR(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z)
{
  // Only use wheel speed measurement of front axle for low slips
  // (this adaptation is only valid for rear wheel drive vehicles)
  if (z[TConfig::PROCESS_AX_MPS2] <= 0.0) {
      // Set Kalman Filter Noise Matrices to rely more on STM for braking maneuvers
      R_.diagonal() = Eigen::Map<Eigen::VectorXd>(
        param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().data(),
        param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().size());
      R_.diagonal()[TConfig::VIRTMEAS_V_STM_1_MPS] = 20.0;
      R_.diagonal()[TConfig::VIRTMEAS_V_STM_2_MPS] = 20.0;
      R_ = R_.array().square();
  } else {
      // Set Kalman Filter Noise Matrices to standard config for accelerating and rolling maneuvers
      R_.diagonal() = Eigen::Map<Eigen::VectorXd>(
        param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().data(),
        param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().size());
      R_ = R_.array().square();
  }
  if (x_[TConfig::STATE_VEL_MPS] <= 2.0) {
      // Set Kalman Filter Noise Matrices to rely more on the wheel speed encoders for low speeds
      R_.diagonal() = Eigen::Map<Eigen::VectorXd>(
        param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().data(),
        param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().size());
      R_.diagonal()[TConfig::VIRTMEAS_V_STM_1_MPS] = 0.01;
      R_.diagonal()[TConfig::VIRTMEAS_V_STM_2_MPS] = 0.01;
      R_ = R_.array().square();
  }
}

template <class TConfig> Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>
  UKF<TConfig>::process_function(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & sigma_point,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE>> & u)
{
  // variables for better readability
  double vel_mps = sigma_point[TConfig::STATE_VEL_MPS];
  double beta_rad = sigma_point[TConfig::STATE_BETA_RAD];
  // sigma point process -- acc. Bechtloff 2018 p. 81
  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> sigma_point_calc;
  sigma_point_calc[TConfig::STATE_VEL_MPS] =
    vel_mps + TConfig::TS *
    (std::cos(beta_rad) * (u[TConfig::PROCESS_AX_MPS2] +
      std::sin(u[TConfig::PROCESS_ROAD_ANGLES_PITCH_RAD]) * u[TConfig::PROCESS_AZ_MPS2]) +
    std::sin(beta_rad) * (u[TConfig::PROCESS_AY_MPS2] -
      std::sin(u[TConfig::PROCESS_ROAD_ANGLES_ROLL_RAD]) *
      std::cos(u[TConfig::PROCESS_ROAD_ANGLES_PITCH_RAD]) * u[TConfig::PROCESS_AZ_MPS2]));
  sigma_point_calc[TConfig::STATE_BETA_RAD] =
    beta_rad + TConfig::TS *
    (-std::sin(beta_rad) * (u[TConfig::PROCESS_AX_MPS2] +
      std::sin(u[TConfig::PROCESS_ROAD_ANGLES_PITCH_RAD]) * u[TConfig::PROCESS_AZ_MPS2]) /
      (vel_mps + 1e-15) +
    (std::cos(beta_rad) * (u[TConfig::PROCESS_AY_MPS2] -
      std::sin(u[TConfig::PROCESS_ROAD_ANGLES_ROLL_RAD]) *
      std::cos(u[TConfig::PROCESS_ROAD_ANGLES_PITCH_RAD]) * u[TConfig::PROCESS_AZ_MPS2])) /
      (vel_mps + 1e-15) -
    u[TConfig::PROCESS_DPSI_RADPS]);
  return sigma_point_calc;
}

template <class TConfig> Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE>
 UKF<TConfig>::get_virtual_measurement(
   const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z)
{
  Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE> z_out;
  // Calculate virtual Measurement Vector
  // speed at front axle middle point
  // acc. Bechtloff "Schätzung des Schwimmwinkels und fahrdynamischer Parameter
  // zur Verbesserung modellbasierter Fahrdynamikregelungen" p. 83
  z_out[TConfig::VIRTMEAS_V_STM_1_MPS] = z[TConfig::UPDATE_OMEGA_WHEEL_FL_RADPS] *
    param_manager_->get_parameter_value("P_VDC_rtire_front_m").as_double() +
    std::cos(z[TConfig::UPDATE_DELTA_RAD]) * z[TConfig::UPDATE_DPSI_RADPS] *
    param_manager_->get_parameter_value("P_VDC_tw_front_m").as_double() * 0.5;
  z_out[TConfig::VIRTMEAS_V_STM_2_MPS] = z[TConfig::UPDATE_OMEGA_WHEEL_FR_RADPS] *
    param_manager_->get_parameter_value("P_VDC_rtire_front_m").as_double() -
    std::cos(z[TConfig::UPDATE_DELTA_RAD]) * z[TConfig::UPDATE_DPSI_RADPS] *
    param_manager_->get_parameter_value("P_VDC_tw_front_m").as_double() * 0.5;
  // STM Forces
  // longitudinal forces
    z_out[TConfig::VIRTMEAS_FTX_F_N] =
      -(2.0 * param_manager_->get_parameter_value("P_VDC_mue_brakes_front").as_double() * 
      (z[TConfig::UPDATE_BRAKE_PRESSURE_FL_PA] + z[TConfig::UPDATE_BRAKE_PRESSURE_FL_PA]) / 2.0 *
      (3.14 * std::pow(param_manager_->get_parameter_value("P_VDC_d_brake_bore_front_m").as_double(), 2) *
      param_manager_->get_parameter_value("P_VDC_r_brake_pads_lever_front_m").as_double() / 4.0)) /
      param_manager_->get_parameter_value("P_VDC_rtire_front_m").as_double();
    z_out[TConfig::VIRTMEAS_FTX_R_N] =
      (-(2.0 * param_manager_->get_parameter_value("P_VDC_mue_brakes_rear").as_double() *
      (z[TConfig::UPDATE_BRAKE_PRESSURE_RL_PA] + z[TConfig::UPDATE_BRAKE_PRESSURE_RR_PA]) / 2.0 *
      (3.14 * std::pow(param_manager_->get_parameter_value("P_VDC_d_brake_bore_rear_m").as_double(), 2) *
      param_manager_->get_parameter_value("P_VDC_r_brake_pads_lever_rear_m").as_double() / 4.0)) +
      z[TConfig::UPDATE_DRIVETRAIN_TORQUE_NM]) /
      param_manager_->get_parameter_value("P_VDC_rtire_rear_m").as_double();
  // lateral forces
  // acc. Bechtloff "Schätzung des Schwimmwinkels und fahrdynamischer Parameter
  // zur Verbesserung modellbasierter Fahrdynamikregelungen" p. 87
  double FY_COG_N =
    z[TConfig::UPDATE_AY_MPS2] * param_manager_->get_parameter_value("P_VDC_mass_kg").as_double();
  double MZZ_COG_Nm =
    z[TConfig::UPDATE_YAW_ACC_RADPS2] *
    param_manager_->get_parameter_value("P_VDC_Izz_kgm2").as_double();
  z_out[TConfig::VIRTMEAS_FTY_F_N] =
    (1 / std::cos(z[TConfig::UPDATE_DELTA_RAD])) * ((FY_COG_N *
    param_manager_->get_parameter_value("P_VDC_l_rear_m").as_double() + MZZ_COG_Nm) / l_m_
    - std::sin(z[TConfig::UPDATE_DELTA_RAD]) * z_out[TConfig::VIRTMEAS_FTX_F_N]);
  z_out[TConfig::VIRTMEAS_FTY_R_N] =
    (FY_COG_N * param_manager_->get_parameter_value("P_VDC_l_front_m").as_double() -
    MZZ_COG_Nm) / l_m_;
  return z_out;
}

template <class TConfig> Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE>
  UKF<TConfig>::transfer_function(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & sigma_point,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z)
{
  // variables for better readability
  double vel_mps = sigma_point[TConfig::STATE_VEL_MPS];
  double beta_rad = sigma_point[TConfig::STATE_BETA_RAD];
  // calculate transformed sigma points acc. Bechtloff 2018 p. 83
  Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE> transformed_sigma_pnt;
  transformed_sigma_pnt[TConfig::VIRTMEAS_V_STM_1_MPS] =
    std::cos(z[TConfig::UPDATE_DELTA_RAD] - beta_rad) * vel_mps +
    std::sin(beta_rad) * param_manager_->get_parameter_value("P_VDC_l_front_m").as_double() *
      z[TConfig::UPDATE_DPSI_RADPS];
  transformed_sigma_pnt[TConfig::VIRTMEAS_V_STM_2_MPS] =
    transformed_sigma_pnt[TConfig::VIRTMEAS_V_STM_1_MPS];
  // calculate load and slip conditions of each tire
  Eigen::Matrix<double, 4, 2> slip_tire_perc =
    tam::core::ssa::UKF<TConfig>::calc_tire_states(sigma_point, z);
  Eigen::Vector<double, 4> slip_tire_long_perc = slip_tire_perc.col(0);
  Eigen::Vector<double, 4> slip_tire_lat_perc = slip_tire_perc.col(1);
  // calculate vertical load of each tire
  Eigen::Vector<double, 4> load_tire_N =
    tam::core::ssa::UKF<TConfig>::calc_tire_loads(sigma_point, z);
  // calculate axle forces considering the specified tire model
  Eigen::Vector<double, 6> axle_forces_N =
    tam::core::ssa::UKF<TConfig>::calc_axle_forces(
      slip_tire_long_perc, slip_tire_lat_perc, load_tire_N);
  // set sigma points to previously calculated axle forces
  transformed_sigma_pnt[TConfig::VIRTMEAS_FTX_F_N] = axle_forces_N(0);
  transformed_sigma_pnt[TConfig::VIRTMEAS_FTX_R_N] = axle_forces_N(3);
  transformed_sigma_pnt[TConfig::VIRTMEAS_FTY_F_N] = axle_forces_N(1);
  transformed_sigma_pnt[TConfig::VIRTMEAS_FTY_R_N] = axle_forces_N(4);
  return transformed_sigma_pnt;
}

template <class TConfig> Eigen::Matrix<double, 4, 2> UKF<TConfig>::calc_tire_states(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & sigma_point,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z)
{
  // variables for better readability
  // velocities COG
  double vel_COG_x_mps =
    sigma_point[TConfig::STATE_VEL_MPS] * std::cos(sigma_point[TConfig::STATE_BETA_RAD]);
  double vel_COG_y_mps =
    sigma_point[TConfig::STATE_VEL_MPS] * std::sin(sigma_point[TConfig::STATE_BETA_RAD]);

  // Calculate Wheel Velocities
  // Transform Vector to Tire Coordinate System
  // transformation matrices COG to wheel position
  Eigen::Vector<double, 4> transform_vx = {
    - param_manager_->get_parameter_value("P_VDC_tw_front_m").as_double() / 2,
    + param_manager_->get_parameter_value("P_VDC_tw_front_m").as_double() / 2,
    - param_manager_->get_parameter_value("P_VDC_tw_rear_m").as_double() / 2,
    + param_manager_->get_parameter_value("P_VDC_tw_rear_m").as_double() / 2
    };
  Eigen::Vector<double, 4> transform_vy = {
    + param_manager_->get_parameter_value("P_VDC_l_front_m").as_double(),
    + param_manager_->get_parameter_value("P_VDC_l_front_m").as_double(),
    - param_manager_->get_parameter_value("P_VDC_l_rear_m").as_double(),
    - param_manager_->get_parameter_value("P_VDC_l_rear_m").as_double()
    };
  Eigen::Vector<double, 4> vel_tire_x_mps = z[TConfig::PROCESS_DPSI_RADPS] * transform_vx +
    Eigen::VectorXd::Constant(4, vel_COG_x_mps);
  Eigen::Vector<double, 4> vel_tire_y_mps = z[TConfig::PROCESS_DPSI_RADPS] * transform_vy +
    Eigen::VectorXd::Constant(4, vel_COG_y_mps);

  // delta_matrix with a parallel steering setup in the front and no rear wheel steering
  Eigen::Vector<double, 4> delta_matrix = {
    z[TConfig::PROCESS_DELTA_RAD], z[TConfig::PROCESS_DELTA_RAD], 0.0, 0.0};
  Eigen::Vector<double, 4> vel_tire_T_x_mps;
  Eigen::Vector<double, 4> vel_tire_T_y_mps;
  vel_tire_T_x_mps = delta_matrix.array().cos() *
    vel_tire_x_mps.array() + delta_matrix.array().sin() * vel_tire_y_mps.array();
  vel_tire_T_y_mps = - delta_matrix.array().sin() *
    vel_tire_x_mps.array() + delta_matrix.array().cos() * vel_tire_y_mps.array();

  // variables for better readability
  // tire_radii
  Eigen::Vector<double, 4> tire_radius_m = {
    param_manager_->get_parameter_value("P_VDC_rtire_front_m").as_double(),
    param_manager_->get_parameter_value("P_VDC_rtire_front_m").as_double(),
    param_manager_->get_parameter_value("P_VDC_rtire_rear_m").as_double(),
    param_manager_->get_parameter_value("P_VDC_rtire_rear_m").as_double()
  };
  // wheelspeed_measurements
  Eigen::Vector<double, 4> wheelspeeds_radps;
  wheelspeeds_radps[TConfig::FRONT_LEFT] = z[TConfig::PROCESS_OMEGA_WHEEL_FL_RADPS];
  wheelspeeds_radps[TConfig::FRONT_RIGHT] = z[TConfig::PROCESS_OMEGA_WHEEL_FR_RADPS];
  wheelspeeds_radps[TConfig::REAR_LEFT] = z[TConfig::PROCESS_OMEGA_WHEEL_RL_RADPS];
  wheelspeeds_radps[TConfig::REAR_RIGHT] = z[TConfig::PROCESS_OMEGA_WHEEL_RR_RADPS];

  // Calculate wheel individual slip
  // Longitudinal
  Eigen::Matrix<double, 4, 2> slip_tire_perc;
  for (int i = 0; i < slip_tire_perc.col(0).size(); ++i) {
    slip_tire_perc(i, 0) = (wheelspeeds_radps(i) * tire_radius_m(i) - vel_tire_T_x_mps(i)) /
        (vel_tire_T_x_mps(i) + 1e-15);
    if (vel_tire_T_x_mps(i) < vel_min_mps_) {
      slip_tire_perc(i, 0) = std::clamp(slip_tire_perc(i, 0), -0.01, 0.01);
    } else {
      slip_tire_perc(i, 0) = std::clamp(slip_tire_perc(i, 0), -1.0, 1.0);
    }
  }
  // Lateral
  for (int i = 0; i < slip_tire_perc.col(1).size(); ++i) {
    slip_tire_perc(i, 1) = - std::atan2(vel_tire_T_y_mps(i), vel_tire_T_x_mps(i));
  }

  return slip_tire_perc;
}

template <class TConfig> Eigen::Vector<double, 4> UKF<TConfig>::calc_tire_loads(
   const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & sigma_point,
   const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z)
{
  // variables for better readability
  // velocities COG
  double vel_COG_x_mps =
    sigma_point[TConfig::STATE_VEL_MPS] * std::cos(sigma_point[TConfig::STATE_BETA_RAD]);
  Eigen::Vector<double, 4> load_tire_N;
  // Calculate Vertical Tire Loads
  // variables for better readability
  // aerodynamic lift coefficient
  Eigen::Vector<double, 4> lift_coeff = {
    param_manager_->get_parameter_value("P_VDC_cL_front").as_double(),
    param_manager_->get_parameter_value("P_VDC_cL_front").as_double(),
    param_manager_->get_parameter_value("P_VDC_cL_rear").as_double(),
    param_manager_->get_parameter_value("P_VDC_cL_rear").as_double()
  };
  // wheel base transform to calculate the static load distribution
  Eigen::Vector<double, 4> wheel_base_transform = {
    param_manager_->get_parameter_value("P_VDC_l_rear_m").as_double(),
    param_manager_->get_parameter_value("P_VDC_l_rear_m").as_double(),
    param_manager_->get_parameter_value("P_VDC_l_front_m").as_double(),
    param_manager_->get_parameter_value("P_VDC_l_front_m").as_double()
  };

  for (int i = 0; i < load_tire_N.size(); ++i) {
    // static load distribution and aerodynamic load
    load_tire_N(i) =
      (param_manager_->get_parameter_value("P_VDC_mass_kg").as_double() *
      z[TConfig::PROCESS_AZ_MPS2] * wheel_base_transform(i) / l_m_ -
      0.5 * param_manager_->get_parameter_value("P_VDC_rho_air_kgpm3").as_double() *
      lift_coeff(i) * 1.00 * vel_COG_x_mps * vel_COG_x_mps) / 2.0;
  }
  return load_tire_N;
}

template <class TConfig> Eigen::Vector<double, 6> UKF<TConfig>::calc_axle_forces(
  const Eigen::Ref<const Eigen::Vector<double, 4>> & slip_tire_long_perc,
  const Eigen::Ref<const Eigen::Vector<double, 4>> & slip_tire_lat_perc,
  const Eigen::Ref<const Eigen::Vector<double, 4>> & load_tire_N)
{
  Eigen::Vector<double, 6> axle_forces;

  // auxilary variables for better readability
  Eigen::VectorXd MF_front_long = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_tire_MF_long_front").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_tire_MF_long_front").as_double_array().size());
  Eigen::VectorXd MF_rear_long = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_tire_MF_long_rear").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_tire_MF_long_rear").as_double_array().size());
  Eigen::VectorXd MF_front_lat = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_tire_MF_lat_front").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_tire_MF_lat_front").as_double_array().size());
  Eigen::VectorXd MF_rear_lat = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_tire_MF_lat_rear").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_tire_MF_lat_rear").as_double_array().size());

  // axle load calculation
  double load_axle_front_N = load_tire_N[TConfig::FRONT_LEFT] + load_tire_N[TConfig::FRONT_RIGHT];
  double load_axle_rear_N = load_tire_N[TConfig::REAR_LEFT] + load_tire_N[TConfig::REAR_RIGHT];
  // axle slip calculation
  double slip_axle_front_long_perc = 
    (slip_tire_long_perc[TConfig::FRONT_LEFT] + slip_tire_long_perc[TConfig::FRONT_RIGHT]) / 2.0;
  double slip_axle_rear_long_perc =
    (slip_tire_long_perc[TConfig::REAR_LEFT] + slip_tire_long_perc[TConfig::REAR_RIGHT]) / 2.0;
  double slip_axle_front_lat_perc =
    (slip_tire_lat_perc[TConfig::FRONT_LEFT] + slip_tire_lat_perc_[TConfig::FRONT_RIGHT]) / 2.0;
  double slip_axle_rear_lat_perc =
    (slip_tire_lat_perc[TConfig::REAR_LEFT] + slip_tire_lat_perc[TConfig::REAR_RIGHT]) / 2.0;

  // axle force calculation
  // front
  axle_forces[0] =
    2.0 * MF_Simple(slip_axle_front_long_perc, load_axle_front_N / 2.0, MF_front_long);
  axle_forces[1] =
    2.0 * MF_Simple(slip_axle_front_lat_perc, load_axle_front_N / 2.0, MF_front_lat);
  axle_forces[2] = load_axle_front_N;
  // rear
  axle_forces[3] =
    2.0 * MF_Simple(slip_axle_rear_long_perc, load_axle_rear_N / 2.0, MF_rear_long);
  axle_forces[4] =
    2.0 * MF_Simple(slip_axle_rear_lat_perc, load_axle_rear_N / 2.0, MF_rear_lat);
  axle_forces[5] = load_axle_rear_N;

  return axle_forces;
}

template <class TConfig> double
  UKF<TConfig>::MF_Simple(
    const double slip, const double load,
    const Eigen::Ref<const Eigen::Vector<double, 4>> & tire_params)
{
  return load * tire_params[2] * std::sin(tire_params[1] * std::atan(tire_params[0] *
    slip - tire_params[3] * (tire_params[0] * slip - std::atan(tire_params[0] * slip))));
}

template <class TConfig> Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>
  UKF<TConfig>::get_state_vector()
{
  return x_;
}

template <class TConfig> Eigen::Matrix<double, 2, 2>
  UKF<TConfig>::get_linear_velocity_covariance()
{
  // auxilary variables for better readability
  double beta = x_[TConfig::STATE_BETA_RAD];
  double vel = x_[TConfig::STATE_VEL_MPS];
  // initialize linear velocity covariance matrix with kalman filter state covariance
  Eigen::Matrix2d P;
  P(0, 0) = P_(TConfig::STATE_VEL_MPS, TConfig::STATE_VEL_MPS);
  P(0, 1) = P_(TConfig::STATE_VEL_MPS, TConfig::STATE_BETA_RAD);
  P(1, 0) = P_(TConfig::STATE_BETA_RAD, TConfig::STATE_VEL_MPS);
  P(1, 1) = P_(TConfig::STATE_BETA_RAD, TConfig::STATE_BETA_RAD);
  // calculate Jacobian to tranform velocity and beta state covariance into vx and vy covariance
  Eigen::Matrix2d J;
  J(0, 0) = std::cos(beta);
  J(0, 1) = -vel * std::sin(beta);
  J(1, 0) = std::sin(beta);
  J(1, 1) = vel * std::cos(beta);
  // return transformed linear velocity covariance
  return J * P * J.transpose();
}

template <class TConfig> std::shared_ptr<tam::interfaces::ParamManagerBase>
  UKF<TConfig>::get_param_handler()
{
  return param_manager_;
}

template <class TConfig> std::map<std::string, double>
  UKF<TConfig>::get_debug()
{
  std::map<std::string, double> debug_output;

  for ( int i = 0; i < TConfig::STATE_VECTOR_SIZE; ++i) {
    debug_output["x_" + std::to_string(i)] = x_[i];
    debug_output["x_pred_" + std::to_string(i)] = x_pred_[i];
    debug_output["x_meas_update_" + std::to_string(i)] = x_meas_update_[i];

    for ( int j = 0; j < TConfig::STATE_VECTOR_SIZE; ++j) {
      debug_output["P_" + std::to_string(i) + std::to_string(j)] = P_(i, j);
      debug_output["P_pred_" + std::to_string(i) + std::to_string(j)] = P_pred_(i, j);
    }

    for ( int j = 0; j < TConfig::VIRTMEAS_VECTOR_SIZE; ++j) {
      debug_output["K_Gain_" + std::to_string(i) + std::to_string(j)] = K_Gain_(i, j);
    }
  }

  for (int i = 0; i < TConfig::VIRTMEAS_VECTOR_SIZE; ++i) {
    for ( int j = 0; j < TConfig::VIRTMEAS_VECTOR_SIZE; ++j) {
      debug_output["P_meas_" + std::to_string(i) + std::to_string(j)] = P_meas_(i, j);
    }
    debug_output["y_" + std::to_string(i)] = y_[i];
    debug_output["z_virt_" + std::to_string(i)] = z_virt_[i];
  }

  for (int i = 0; i < 4; i++) {
    debug_output["slip_tire_long_perc_" + std::to_string(i)] = slip_tire_long_perc_[i];
    debug_output["slip_tire_lat_perc_" + std::to_string(i)] = slip_tire_lat_perc_[i];
    debug_output["load_tire_N_" + std::to_string(i)] = load_tire_N_[i];
  }

  debug_output["force_axle_front_N_long"] = force_axle_front_N_[0];
  debug_output["force_axle_rear_N_long"] = force_axle_rear_N_[0];
  debug_output["force_axle_front_N_lat"] = force_axle_front_N_[1];
  debug_output["force_axle_rear_N_lat"] = force_axle_rear_N_[1];
  debug_output["force_axle_front_N_vert"] = force_axle_front_N_[2];
  debug_output["force_axle_rear_N_vert"] = force_axle_rear_N_[2];

  return debug_output;
}
}  // namespace tam::core::ssa
