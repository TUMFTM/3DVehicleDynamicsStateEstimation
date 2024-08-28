// Copyright 2023 Marcel Weinmann
#pragma once
#include "state_estimation_cpp/filter/kalman_filter/ekf.hpp"

namespace tam::core::state
{
template <typename TConfig> EKF<TConfig>::EKF()
{
  if constexpr (!ekf::HasStateThetaRad<TConfig>)
  {
    // Initialize EKF in 2D
    // ensure that all matrices are set to zero
    A_.setZero();
    B_.setZero();

    param_manager_->declare_parameter(
      "P_VDC_MeasCov",
      std::vector<double>{0.4, 0.4, 0.4, 0.4, 0.2, 0.1, 0.1, 0.1, 0.04, 0.04, 0.01, 0.01, 0.01, 0.01,
      0.05, 0.05, 0.1, 0.1, 0.1, 2.0}, tam::types::param::ParameterType::DOUBLE_ARRAY, "");

    param_manager_->declare_parameter(
      "P_VDC_InputCov",
      std::vector<double>{0.00015, 0.5, 1.8}, tam::types::param::ParameterType::DOUBLE_ARRAY, "");

    param_manager_->declare_parameter(
      "P_VDC_OutlierBounds",
      std::vector<double>{2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3,
      2.0, 1.0, 2.0, 1.0, 2.0, 2.0}, tam::types::param::ParameterType::DOUBLE_ARRAY, "");

    param_manager_->declare_parameter(
      "P_VDC_MahalanobisCovariance",
      std::vector<double>{3.5, 2.0, 3.5, 2.0, 3.5, 2.0, 3.5, 2.0, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3,
      2.0, 1.0, 2.0, 1.0, 2.0, 5.0}, tam::types::param::ParameterType::DOUBLE_ARRAY, "");

    // Initial guess for the accel and gyro offset
    param_manager_->declare_parameter(
      "P_VDC_InitialBias", std::vector<double>{0.0, 0.0, 0.0},
      tam::types::param::ParameterType::DOUBLE_ARRAY, "");

    // set the H_full matrix (this matrix would fuse all measurements)
    // first set the rows that correspond to the localization measurements
    for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
      H_full_.row(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_POS_X_M)     << 1, 0, 0, 0, 0;
      H_full_.row(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_POS_Y_M)     << 0, 1, 0, 0, 0;
    }

    // in the next step we set the rows that correspond to the orientation measurements
    for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
      H_full_.row(i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                  + TConfig::MEASUREMENT_PSI_RAD)     << 0, 0, 1, 0, 0;
    }

    // in the next step we set the rows that correspond to the velocity measurements
    for (int i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
      H_full_.row(i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                  + TConfig::MEASUREMENT_VX_MPS)      << 0, 0, 0, 1, 0;
      H_full_.row(i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                  + TConfig::MEASUREMENT_VY_MPS)      << 0, 0, 0, 0, 1;
    }
  } else {
    // Initialize EKF in 3D
    // ensure that all matrices are set to zero
    A_.setZero();
    B_.setZero();

    param_manager_->declare_parameter(
      "P_VDC_MeasCov",
      std::vector<double>{0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4,
                          0.1, 0.1, 0.004, 0.1, 0.1, 0.004, 0.1, 0.1, 0.04, 0.1, 0.1, 0.04,
                          0.1, 0.1, 0.004, 0.1, 0.1, 0.004, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                          0.1, 2.0, 0.1},
      tam::types::param::ParameterType::DOUBLE_ARRAY, "");

    param_manager_->declare_parameter(
      "P_VDC_InputCov",
      std::vector<double>{0.00015, 0.00015, 0.00015, 0.5, 1.8, 2.0},
      tam::types::param::ParameterType::DOUBLE_ARRAY, "");

    param_manager_->declare_parameter(
      "P_VDC_OutlierBounds",
      std::vector<double>{2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                          0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3,
                          0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 2.0, 1.0, 2.0, 2.0, 1.0, 2.0,
                          2.0, 2.0, 2.0},
      tam::types::param::ParameterType::DOUBLE_ARRAY, "");

    param_manager_->declare_parameter(
      "P_VDC_MahalanobisCovariance",
      std::vector<double>{3.5, 2.0, 4.0, 3.5, 2.0, 4.0, 3.5, 2.0, 4.0, 3.5, 2.0, 4.0,
                          0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3,
                          0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 2.0, 1.0, 2.0, 2.0, 1.0, 2.0,
                          2.0, 5.0, 5.0},
      tam::types::param::ParameterType::DOUBLE_ARRAY, "");

    // Initial guess for the accel and gyro offset
    param_manager_->declare_parameter(
      "P_VDC_InitialBias", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      tam::types::param::ParameterType::DOUBLE_ARRAY, "");

    // set the H_full matrix (this matrix would fuse all measurements)
    // first set the rows that correspond to the localization measurements
    for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
      H_full_.row(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_POS_X_M)     << 1, 0, 0, 0, 0, 0, 0, 0, 0;
      H_full_.row(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_POS_Y_M)     << 0, 1, 0, 0, 0, 0, 0, 0, 0;
      H_full_.row(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_POS_Z_M)     << 0, 0, 1, 0, 0, 0, 0, 0, 0;
    }

    // in the next step we set the rows that correspond to the orientation measurements
    for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
      H_full_.row(i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                  + TConfig::MEASUREMENT_PHI_RAD)     << 0, 0, 0, 1, 0, 0, 0, 0, 0;
      H_full_.row(i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                  + TConfig::MEASUREMENT_THETA_RAD)   << 0, 0, 0, 0, 1, 0, 0, 0, 0;
      H_full_.row(i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                  + TConfig::MEASUREMENT_PSI_RAD)     << 0, 0, 0, 0, 0, 1, 0, 0, 0;
    }

    // in the next step we set the rows that correspond to the velocity measurements
    for (int i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
      H_full_.row(i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                  + TConfig::MEASUREMENT_VX_MPS)      << 0, 0, 0, 0, 0, 0, 1, 0, 0;
      H_full_.row(i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                  + TConfig::MEASUREMENT_VY_MPS)      << 0, 0, 0, 0, 0, 0, 0, 1, 0;
      H_full_.row(i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
                  + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                  + TConfig::MEASUREMENT_VZ_MPS)      << 0, 0, 0, 0, 0, 0, 0, 0, 1;
    }
  }
}

/**
 * @brief set the covariance matricies defined in the param handler
 */
template <class TConfig> void EKF<TConfig>::set_covariance_matricies(void)
{
  // set the Measurement Noise Covariance Matrix
  R_.diagonal() = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_MeasCov").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_MeasCov").as_double_array().size());

  R_adaptive_ = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_MeasCov").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_MeasCov").as_double_array().size());

  // set the Process Noise Covariance Matrix
  Q_.diagonal() = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_InputCov").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_InputCov").as_double_array().size());

  // set the outlier bound vector to only load the data from the param manager once
  outlier_bound_ = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_OutlierBounds").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_OutlierBounds").as_double_array().size());

  // set the mahalanobis covariance matrix from param manager
  mahalanobis_covariance_ = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_MahalanobisCovariance").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_MahalanobisCovariance").as_double_array().size());
}

/**
 * @brief Prediction step of the EKF
 *
 * @param[in] u             - Input vector:
 *                            [dPsi_radps, ax_mps2, ay_mps2]
 */
template <typename TConfig> void EKF<TConfig>::predict(
  const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u)
{
  if constexpr (!ekf::HasStateThetaRad<TConfig>) {
    // Prediction Step of the 2D EKF
    // define the following functions for better readability
    // previous state vector
    double psi_rad    = x_[TConfig::STATE_PSI_RAD];
    double vx_mps     = x_[TConfig::STATE_VX_MPS];
    double vy_mps     = x_[TConfig::STATE_VY_MPS];

    // new input vector
    double dPsi_radps = u[TConfig::INPUT_DPSI_RADPS];
    double ax_mps2    = u[TConfig::INPUT_AX_MPS2];
    double ay_mps2    = u[TConfig::INPUT_AY_MPS2];

    // construct the Jacobian matrix consisting of partial derivatives with respect to the
    // system state from the system update equation
    A_ << 0, 0, (-std::sin(psi_rad) * vx_mps - std::cos(psi_rad) * vy_mps),
                  std::cos(psi_rad), -std::sin(psi_rad),
          0, 0, (std::cos(psi_rad) * vx_mps - std::sin(psi_rad) * vy_mps),
                  std::sin(psi_rad), std::cos(psi_rad),
          0, 0, 0, 0, 0,
          0, 0, 0, 0, dPsi_radps,
          0, 0, 0, -dPsi_radps, 0;

    A_ = Eigen::MatrixXd::Identity(TConfig::STATE_VECTOR_SIZE,
                                  TConfig::STATE_VECTOR_SIZE)
        + A_ * TConfig::TS;

    // Jacobian matrix B consisting of partial derivatives with respect to the
    // system input from the system update equation
    if (param_manager_->get_parameter_value("P_VDC_EnableInputCrossCorrelation").as_bool()) {
      B_ << 0, 0, 0,
            0, 0, 0,
            1, 0, 0,
            vy_mps, 1, 0,
            -vx_mps, 0, 1;
    } else {
      B_ << 0, 0, 0,
            0, 0, 0,
            1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    }

    B_ = TConfig::TS * B_;

    // fuse the process noise covariance matrix with
    // the linearized version of the system input vector (non-additive noise)
    Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE,
      TConfig::STATE_VECTOR_SIZE> Q_lin = B_ * Q_ * B_.transpose();

    // State Transition:
    // Rigid body model using accelerations and yaw rate as inputs and applying simple forward
    // integration in vehicle coordinate frame to obtain the velocities. The latter are then
    // used to apply forward integration in global cartesian coordinates to obtain the
    // position. Using the ENU convention (0 degrees heading is east (x-axis)).
    Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> dx;
    dx << std::cos(psi_rad) * vx_mps - std::sin(psi_rad) * vy_mps,
          std::sin(psi_rad) * vx_mps + std::cos(psi_rad) * vy_mps,
          dPsi_radps,
          ax_mps2 + dPsi_radps * vy_mps,
          ay_mps2 - dPsi_radps * vx_mps;

    // integrate over on timestep
    x_pred_ = x_ + TConfig::TS * dx;

    // Normalize the angles of every angle in the state transition vector
    for (int i = 0; i < TConfig::STATE_VECTOR_SIZE; i++) {
      if (TConfig::STATE_VECTOR_ANGLE_INDICATOR[i] == true) {
        x_pred_[i] = tam::helpers::geometry::normalize_angle(x_pred_[i]);
      }
    }

    // Predict the state covariance
    P_pred_ = A_ * P_ * A_.transpose() + Q_lin;
  } else {
    // Prediction Step of the 3D EKF
    // define the following functions for better readability
    // previous state vector
    double phi_rad      = x_[TConfig::STATE_PHI_RAD];
    double theta_rad    = x_[TConfig::STATE_THETA_RAD];
    double psi_rad      = x_[TConfig::STATE_PSI_RAD];
    double vx_mps       = x_[TConfig::STATE_VX_MPS];
    double vy_mps       = x_[TConfig::STATE_VY_MPS];
    double vz_mps       = x_[TConfig::STATE_VZ_MPS];

    // new input vector
    double dPhi_radps   = u[TConfig::INPUT_DPHI_RADPS];
    double dTheta_radps = u[TConfig::INPUT_DTHETA_RADPS];
    double dPsi_radps   = u[TConfig::INPUT_DPSI_RADPS];
    double ax_mps2      = u[TConfig::INPUT_AX_MPS2];
    double ay_mps2      = u[TConfig::INPUT_AY_MPS2];
    double az_mps2      = u[TConfig::INPUT_AZ_MPS2];

    // precalculate sin and cos for all angles to increase efficiency
    double sphi   = std::sin(phi_rad),   cphi   = std::cos(phi_rad);
    double stheta = std::sin(theta_rad), ctheta = std::cos(theta_rad), ttheta = std::tan(theta_rad);
    double spsi   = std::sin(psi_rad),   cpsi   = std::cos(psi_rad);

    // State Transition:
    // Rigid body model using accelerations and angular velocities as inputs
    // and applying simple forward integration in vehicle coordinate frame to obtain the velocities.
    // The latter are then used to apply forward integration in global cartesian coordinates to obtain
    // the position. Using the ENU convention (0 degrees heading is east (x-axis)).
    
    // rotate velocities from the vehicle frame into the enu frame to integrate over them
    double dx_pos_x = ctheta * cpsi * vx_mps + (sphi * stheta * cpsi - cphi * spsi) * vy_mps
                      + (cphi * stheta * cpsi + sphi * spsi) * vz_mps;
    double dx_pos_y = ctheta * spsi * vx_mps + (sphi * stheta * spsi + cphi * cpsi) * vy_mps
                      + (cphi * stheta * spsi - sphi * cpsi) * vz_mps;
    double dx_pos_z = - stheta * vx_mps + sphi * ctheta * vy_mps + cphi * ctheta * vz_mps;
      
    // To integrate over the angluar velocity we have to rotate the data into the enu frame
    // https://rotations.berkeley.edu/strapdown-inertial-navigation/
    double dx_phi   = dPhi_radps + sphi * ttheta * dTheta_radps + cphi * ttheta * dPsi_radps;
    double dx_theta = cphi * dTheta_radps - sphi * dPsi_radps;
    double dx_psi   = sphi / ctheta * dTheta_radps + cphi / ctheta * dPsi_radps;

    // compensate gravity and centrifual accelerations for the imu measurements to get velocities
    // https://acl.kaist.ac.kr/wp-content/uploads/2021/10/2013partd_OJW.pdf (equation 33)
    // |v_x'|     |omega_x|     |v_x|   |a_meas_x|       |    stheata   |
    // |v_y'| = - |omega_y|   x |v_y| + |a_meas_y| + g x |-sphi x ctheta| 
    // |v_z'|     |omega_z|_x   |v_z|   |a_meas_z|       |-cphi x ctheta|
    double dx_vx    = ax_mps2 + dPsi_radps * vy_mps - dTheta_radps * vz_mps
                      + stheta * tam::constants::g_earth;
    double dx_vy    = ay_mps2 - dPsi_radps * vx_mps + dPhi_radps * vz_mps
                      - sphi * ctheta * tam::constants::g_earth;
    double dx_vz    = az_mps2 + dTheta_radps * vx_mps - dPhi_radps * vy_mps
                      - cphi * ctheta * tam::constants::g_earth;

    Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> dx;
    dx << dx_pos_x,
          dx_pos_y,
          dx_pos_z,
          dx_phi,
          dx_theta,
          dx_psi,
          dx_vx,
          dx_vy,
          dx_vz;

    // partial derivatives of Pos_x_m
    double dx_pos_x_wrt_phi   = (cphi * stheta * cpsi + sphi * spsi) * vy_mps
                                + (- sphi * stheta * cpsi + cphi * spsi) * vz_mps;
    double dx_pos_x_wrt_theta = - stheta * cpsi * vx_mps + sphi * ctheta * cpsi * vy_mps
                                + cphi * ctheta * cpsi * vz_mps;
    double dx_pos_x_wrt_psi   = - ctheta * spsi * vx_mps + (- sphi * stheta * spsi - cphi * cpsi)
                                * vy_mps + (- cphi * stheta * spsi + sphi * cpsi) * vz_mps;
    double dx_pos_x_wrt_vx    = ctheta * cpsi;
    double dx_pos_x_wrt_vy    = sphi * stheta * cpsi - cphi * spsi;
    double dx_pos_x_wrt_vz    = cphi * stheta * cpsi + sphi * spsi;

    // partial derivatives of Pos_y_m
    double dx_pos_y_wrt_phi   = (cphi * stheta * spsi - sphi * cpsi) * vy_mps
                                + (- sphi * stheta * spsi - cphi * cpsi) * vz_mps;
    double dx_pos_y_wrt_theta = - stheta * spsi * vx_mps + sphi * ctheta * vy_mps
                                + cphi * ctheta * spsi * vz_mps;
    double dx_pos_y_wrt_psi   = ctheta * cpsi * vx_mps + (sphi * stheta * cpsi - cphi * spsi) * vy_mps
                                + (cphi * stheta * cpsi + sphi * spsi) * vz_mps;
    double dx_pos_y_wrt_vx    = ctheta * spsi;
    double dx_pos_y_wrt_vy    = sphi * stheta * spsi + cphi * cpsi;
    double dx_pos_y_wrt_vz    = cphi * stheta * spsi - sphi * cpsi;

    // partial derivatives of Pos_z_m
    double dx_pos_z_wrt_phi   = cphi * ctheta * vy_mps - sphi * ctheta * vz_mps;
    double dx_pos_z_wrt_theta = - ctheta * vx_mps - sphi * stheta * vy_mps - cphi * stheta * vz_mps;
    double dx_pos_z_wrt_psi   = 0.0;
    double dx_pos_z_wrt_vx    = - stheta;
    double dx_pos_z_wrt_vy    = sphi * ctheta;
    double dx_pos_z_wrt_vz    = cphi * ctheta;

    // partial derivatives of Phi_rad
    double dx_phi_wrt_phi     = cphi * ttheta * dTheta_radps - sphi * ttheta * dPsi_radps;
    double dx_phi_wrt_theta   = (sphi * dTheta_radps + cphi * dPsi_radps) / (ctheta * ctheta);
    double dx_phi_wrt_psi     = 0.0;

    // partial derivatives of Theta_rad
    double dx_theta_wrt_phi   = - sphi * dTheta_radps - cphi * dPsi_radps;
    double dx_theta_wrt_theta = 0.0;
    double dx_theta_wrt_psi   = 0.0;
    
    // partial derivatives of Psi_rad
    double dx_psi_wrt_phi     = cphi / ctheta * dTheta_radps - sphi / ctheta * dPsi_radps;
    double dx_psi_wrt_theta   = (sphi * dTheta_radps + cphi * dPsi_radps) * stheta / ctheta / ctheta;
    double dx_psi_wrt_psi     = 0.0;

    // partial derivatives of vx_mps
    double dx_vx_wrt_vx      = 0.0;
    double dx_vx_wrt_vy      = dPsi_radps;
    double dx_vx_wrt_vz      = - dTheta_radps;
      
    // partial derivatives of vy_mps
    double dx_vy_wrt_vx      = - dPsi_radps;
    double dx_vy_wrt_vy      = 0.0;
    double dx_vy_wrt_vz      = dPhi_radps;

    // partial derivatives of vz_mps
    double dx_vz_wrt_vx      = dTheta_radps;
    double dx_vz_wrt_vy      = - dPhi_radps;
    double dx_vz_wrt_vz      = 0.0;

    // construct the Jacobian matrix consisting of partial derivatives with respect to the
    // system state from the system update equation
    A_ << 0, 0, 0, dx_pos_x_wrt_phi, dx_pos_x_wrt_theta, dx_pos_x_wrt_psi, dx_pos_x_wrt_vx, dx_pos_x_wrt_vy, dx_pos_x_wrt_vz,
          0, 0, 0, dx_pos_y_wrt_phi, dx_pos_y_wrt_theta, dx_pos_y_wrt_psi, dx_pos_y_wrt_vx, dx_pos_y_wrt_vy, dx_pos_y_wrt_vz,
          0, 0, 0, dx_pos_z_wrt_phi, dx_pos_z_wrt_theta, dx_pos_z_wrt_psi, dx_pos_z_wrt_vx, dx_pos_z_wrt_vy, dx_pos_z_wrt_vz,
          0, 0, 0, dx_phi_wrt_phi,   dx_phi_wrt_theta,   dx_phi_wrt_psi,   0, 0, 0,
          0, 0, 0, dx_theta_wrt_phi, dx_theta_wrt_theta, dx_theta_wrt_psi, 0, 0, 0,
          0, 0, 0, dx_psi_wrt_phi,   dx_psi_wrt_theta,   dx_psi_wrt_psi,   0, 0, 0,
          0, 0, 0, 0, 0, 0, dx_vx_wrt_vx, dx_vx_wrt_vy, dx_vx_wrt_vz,
          0, 0, 0, 0, 0, 0, dx_vy_wrt_vx, dx_vy_wrt_vy, dx_vy_wrt_vz,
          0, 0, 0, 0, 0, 0, dx_vz_wrt_vx, dx_vz_wrt_vy, dx_vz_wrt_vz;

    A_ = Eigen::MatrixXd::Identity(TConfig::STATE_VECTOR_SIZE,
                                  TConfig::STATE_VECTOR_SIZE)
        + A_ * TConfig::TS;

    // Jacobian matrix B consisting of partial derivatives with respect to the
    // system input from the system update equation
    if (param_manager_->get_parameter_value("P_VDC_EnableInputCrossCorrelation").as_bool()) {
      // partial derivatives of Phi_rad
      double dx_phi_wrt_dphi     = 1.0;
      double dx_phi_wrt_dtheta   = sphi * ttheta;
      double dx_phi_wrt_dpsi     = cphi * ttheta;
    
      // partial derivatives of Theta_rad
      double dx_theta_wrt_dphi   = 0.0;
      double dx_theta_wrt_dtheta = cphi;
      double dx_theta_wrt_dpsi   = - sphi;
      // partial derivatives of Psi_rad
      double dx_psi_wrt_dphi     = 0.0;
      double dx_psi_wrt_dtheta   = sphi / ctheta;
      double dx_psi_wrt_dpsi     = cphi / ctheta;

      // partial derivatives of vx_mps
      double dx_vx_wrt_dphi     = 0.0;
      double dx_vx_wrt_dtheta   = - vz_mps;
      double dx_vx_wrt_dpsi     = vy_mps;

      // partial derivatives of vy_mps
      double dx_vy_wrt_dphi     = vz_mps;
      double dx_vy_wrt_dtheta   = 0.0;
      double dx_vy_wrt_dpsi     = - vx_mps;

      // partial derivatives of vz_mps
      double dx_vz_wrt_dphi     = - vy_mps;
      double dx_vz_wrt_dtheta   = vx_mps;
      double dx_vz_wrt_dpsi     = 0.0;

      B_ << 0,                 0,                   0,                 0, 0, 0,
            0,                 0,                   0,                 0, 0, 0,
            0,                 0,                   0,                 0, 0, 0,
            dx_phi_wrt_dphi,   dx_phi_wrt_dtheta,   dx_phi_wrt_dpsi,   0, 0, 0,
            dx_theta_wrt_dphi, dx_theta_wrt_dtheta, dx_theta_wrt_dpsi, 0, 0, 0,
            dx_psi_wrt_dphi,   dx_psi_wrt_dtheta,   dx_psi_wrt_dpsi,   0, 0, 0,
            dx_vx_wrt_dphi,    dx_vx_wrt_dtheta,    dx_vx_wrt_dpsi,    1, 0, 0,
            dx_vy_wrt_dphi,    dx_vy_wrt_dtheta,    dx_vy_wrt_dpsi,    0, 1, 0,
            dx_vz_wrt_dphi,    dx_vz_wrt_dtheta,    dx_vz_wrt_dpsi,    0, 0, 1;
      } else {
        B_ << 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0,
              1, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0,
              0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 1;
      }

      B_ = TConfig::TS * B_;

      // fuse the process noise covariance matrix with
      // the linearized version of the system input vector (non-additive noise)
      Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE,
        TConfig::STATE_VECTOR_SIZE> Q_lin = B_ * Q_ * B_.transpose();

      // use euler integration to integrate over one timestep
      x_pred_ = x_ + TConfig::TS * dx;

      // Normalize the angles of every angle in the state transition vector
      for (int i = 0; i < TConfig::STATE_VECTOR_SIZE; i++) {
        if (TConfig::STATE_VECTOR_ANGLE_INDICATOR[i] == true) {
          x_pred_[i] = tam::helpers::geometry::normalize_angle(x_pred_[i]);
        }
      }

      // Predict the state covariance
      P_pred_ = A_ * P_ * A_.transpose() + Q_lin;
  }
}

/**
 * @brief Correction step of the EKF
 *
 * @param[in] z             - Measurement vector
 */
template <class TConfig> void EKF<TConfig>::update(
  const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & z)
{
  // calculate the innovation/ measurement residuals
  residuals_raw_ = z - H_ * x_pred_;

  // normalize all residual angles
  for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
    for (int j = 0; j < TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE; j++) {
        residuals_raw_[TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                       + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + j]
          = tam::helpers::geometry::normalize_angle(
              residuals_raw_[TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                             + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + j]);
    }
  }

  // Calculate the innovation (residual) covariance S
  Eigen::Matrix<double, TConfig::MEASUREMENT_VECTOR_SIZE, TConfig::MEASUREMENT_VECTOR_SIZE> S
    = H_ * P_pred_ * H_.transpose() + R_;

  // Calculate the Kalman Gain K
  K_ = P_pred_ * H_.transpose() * S.inverse();

  // perform the outlier filtering
  // the maximum allowed residum is defined in the variable P_VDC_OutlierBounds
  residuals_raw_ = residuals_raw_.cwiseProduct(fusion_vec_);
  if (param_manager_->get_parameter_value("P_VDC_EnableMahalanobisOutlierDetection").as_bool()) {
    residuals_ = tam::core::state::outlier_detection::mahalanobis_outlier_detection<TConfig>(
      residuals_raw_, mahalanobis_covariance_, x_);
  } else {
    residuals_ = tam::core::state::outlier_detection::box_outlier_detection(
      residuals_raw_, outlier_bound_);
  }

  // set the residual to zero if the sensor was not updated or is not valid
  residuals_ = residuals_.cwiseProduct(fusion_vec_);

  // update the state vector w.r.t the Kalman Gain
  x_ = x_pred_ + K_ * residuals_;

  // normalize all angles of the state vector
  for (int i = 0; i < TConfig::STATE_VECTOR_SIZE; i++) {
    if (TConfig::STATE_VECTOR_ANGLE_INDICATOR[i] == true) {
      x_[i] = tam::helpers::geometry::normalize_angle(x_[i]);
    }
  }

  // update the state covariance matrix
  P_ = (Eigen::MatrixXd::Identity(TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE)
       - K_ * H_) * P_pred_;
}

/**
 * @brief get all debug values of the kalman filter
 *
 * @param[out]              - std::map<std::string, double>:
 *                            residuals of the Extended Kalman Filter
 */
template <typename TConfig>
std::map<std::string, double> EKF<TConfig>::get_debug(void)
{
  std::map<std::string, double> debug_output;
  if constexpr (!ekf::HasStateThetaRad<TConfig>)
  {
    // get the debug outputs of the 2D EKF

    // Iterate through all position residuals
    for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
      // residual in x in meter
      debug_output["res_loc_" + std::to_string(i + 1) + "_x_m"] =
        residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_POS_X_M];

      // Measurement variance of loc in x
      debug_output["R_loc_" + std::to_string(i + 1) + "_x"] =
        R_(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE 
              + TConfig::MEASUREMENT_POS_X_M,
          i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
              + TConfig::MEASUREMENT_POS_X_M);

      // residual in y in meter
      debug_output["res_loc_" + std::to_string(i + 1) + "_y_m"] =
        residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_POS_Y_M];

      // Measurement variance of loc in y
      debug_output["R_loc_" + std::to_string(i + 1) + "_y"] =
        R_(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE 
              + TConfig::MEASUREMENT_POS_Y_M,
          i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
              + TConfig::MEASUREMENT_POS_Y_M);

      // residual in s in meter
      debug_output["res_loc_" + std::to_string(i + 1) + "_s_m"] =
        std::cos(x_[TConfig::STATE_PSI_RAD])
          * residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE]
        + std::sin(x_[TConfig::STATE_PSI_RAD])
          * residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + 1];

      // residual in d in meter
      debug_output["res_loc_" + std::to_string(i + 1) + "_d_m"] =
        - std::sin(x_[TConfig::STATE_PSI_RAD])
          * residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE]
        + std::cos(x_[TConfig::STATE_PSI_RAD])
          * residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + 1];
    }

    // Iterate through all orientation residuals
    for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
      // residual in psi in rad
      debug_output["res_loc_" + std::to_string(i + 1) + "_psi_rad"] =
        residuals_raw_[i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                      + TConfig::MEASUREMENT_PSI_RAD];

      // Kalman Gain of loc in psi
      debug_output["R_loc_" + std::to_string(i + 1) + "_psi"] =
        R_(i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE 
              + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
              + TConfig::MEASUREMENT_PSI_RAD,
          i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
              + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
              + TConfig::MEASUREMENT_PSI_RAD);
    }

    // Iterate through all linear velocity residuals
    for (int i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
      // residual in vx in m/s
      debug_output["res_lin_vel_" + std::to_string(i + 1) + "_vx_mps"] =
        residuals_raw_[i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                      + TConfig::MEASUREMENT_VX_MPS];

      // residual in vy in m/s
      debug_output["res_lin_vel_" + std::to_string(i + 1) + "_vy_mps"] =
        residuals_raw_[i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                      + TConfig::MEASUREMENT_VY_MPS];
    }
  } else {
    // get the debug outputs of the 3D EKF
    // Iterate through all localization residuals
    for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
      // residual in x in meter
      debug_output["res_loc_" + std::to_string(i + 1) + "_x_m"] =
        residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_POS_X_M];

      // Measurement variance of loc in x
      debug_output["R_loc_" + std::to_string(i + 1) + "_x"] =
        R_(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
              + TConfig::MEASUREMENT_POS_X_M,
          i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
              + TConfig::MEASUREMENT_POS_X_M);

      // residual in y in meter
      debug_output["res_loc_" + std::to_string(i + 1) + "_y_m"] =
        residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_POS_Y_M];

      // Measurement variance of loc in y
      debug_output["R_loc_" + std::to_string(i + 1) + "_y"] =
        R_(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
              + TConfig::MEASUREMENT_POS_Y_M,
          i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
              + TConfig::MEASUREMENT_POS_Y_M);

      // residual in z in meter
      debug_output["res_loc_" + std::to_string(i + 1) + "_z_m"] =
        residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_POS_Z_M];

      // Measurement variance of loc in y
      debug_output["R_loc_" + std::to_string(i + 1) + "_z"] =
        R_(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
              + TConfig::MEASUREMENT_POS_Z_M,
          i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
              + TConfig::MEASUREMENT_POS_Z_M);

      // residual in s in meter
      debug_output["res_loc_" + std::to_string(i + 1) + "_s_m"] =
        std::cos(x_[TConfig::STATE_PSI_RAD])
          * residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE]
        + std::sin(x_[TConfig::STATE_PSI_RAD])
          * residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + 1];

      // residual in d in meter
      debug_output["res_loc_" + std::to_string(i + 1) + "_d_m"] =
        - std::sin(x_[TConfig::STATE_PSI_RAD])
          * residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE]
        + std::cos(x_[TConfig::STATE_PSI_RAD])
          * residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + 1];
    }

    // Iterate through all orientation residuals
    for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
      // residual in phi in rad
      debug_output["res_loc_" + std::to_string(i + 1) + "_phi_rad"] =
        residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                      + TConfig::MEASUREMENT_PHI_RAD];

      // residual in theta in rad
      debug_output["res_loc_" + std::to_string(i + 1) + "_theta_rad"] =
        residuals_raw_[i * TConfig::POS_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                      + TConfig::MEASUREMENT_THETA_RAD];

      // residual in psi in rad
      debug_output["res_loc_" + std::to_string(i + 1) + "_psi_rad"] =
        residuals_raw_[i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                      + TConfig::MEASUREMENT_PSI_RAD];

      // Kalman Gain of loc in psi
      debug_output["R_loc_" + std::to_string(i + 1) + "_psi"] =
        R_(i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
              + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
              + TConfig::MEASUREMENT_PSI_RAD,
          i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE
              + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
              + TConfig::MEASUREMENT_PSI_RAD);
    }

    // Iterate through all linear velocity residuals
    for (int i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
      // residual in vx in m/s
      debug_output["res_lin_vel_" + std::to_string(i + 1) + "_vx_mps"] =
        residuals_raw_[i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                      + TConfig::MEASUREMENT_VX_MPS];

      // residual in vy in m/s
      debug_output["res_lin_vel_" + std::to_string(i + 1) + "_vy_mps"] =
        residuals_raw_[i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                      + TConfig::MEASUREMENT_VY_MPS];

      // residual in vz in m/s
      debug_output["res_lin_vel_" + std::to_string(i + 1) + "_vz_mps"] =
        residuals_raw_[i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE
                      + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                      + TConfig::MEASUREMENT_VZ_MPS];
    }
  }
  return debug_output;
}
}  // namespace tam::core::state
