// Copyright 2023 Marcel Weinmann
#pragma once
#include "imu_handler/imu_handler.hpp"
namespace tam::core::state
{
template <class TConfig> IMUHandler<TConfig>::IMUHandler()
{
  // set the imu biases to zero
  imu_bias_.setZero();

  // param_manager
  param_manager_ = std::make_shared<tam::core::ParamManager>();

  for (int i = 0; i < TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT; ++i) {
    param_manager_->declare_parameter(
      "P_VDC_IMU" + std::to_string(i + 1) + "_filter_coefficients",
      std::vector<double>{0.07722183762197628, 0.24522338986219644, 0.34256086202951863,
      0.24522338986219644, 0.07722183762197628},
      tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  }
}

template <class TConfig> const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>
  IMUHandler<TConfig>::update_input_vector(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE
      * (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>> & u_raw,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE
      * (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>> & u_fusion_vec)
{
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u;
  if constexpr (TConfig::INPUT_VECTOR_SIZE == 6) {
    // average the imu data
    Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>
      u_average = IMUHandler<TConfig>::average_input_vector(
                    u_raw, u_fusion_vec);
    u = u_average - imu_bias_;
  } else if constexpr (TConfig::INPUT_VECTOR_SIZE == 3) {
    // average the imu data
    u = IMUHandler<TConfig>::average_input_vector(
        u_raw, u_fusion_vec);

    // compensate the banking angle for dpsi_rad
    u[TConfig::INPUT_DPSI_RADPS] /= std::cos(-road_angles_.x);

    // add ay_calibration_offset to all ay_mps2 measurements
    u[TConfig::INPUT_AY_MPS2] -=
      imu_bias_[TConfig::INPUT_AY_MPS2];

    // compensate banking for ay_mps2
    u[TConfig::INPUT_AY_MPS2]
      *= std::sin(-road_angles_.x) * std::tan(-road_angles_.x) + std::cos(-road_angles_.x);

    // compensate gravity for ay_mps2
    u[TConfig::INPUT_AY_MPS2]
      += std::tan(-road_angles_.x) * tam::constants::g_earth;
  } else {
    static_assert(TConfig::INPUT_VECTOR_SIZE == 3 || TConfig::INPUT_VECTOR_SIZE == 6,
      "[IMUHandler]: invalid INPUT_VECTOR_SIZE");
  }
  return u;
}

template <class TConfig> const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>
  IMUHandler<TConfig>::average_input_vector(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE
    * (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>> & u_raw,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE
    * (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>> & u_fusion_vec)
{
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> num_valid_imus;
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u;

  num_valid_imus.setZero();
  u.setZero();

  // only use valid measurements
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE * (TConfig::NUM_IMU_MEASUREMENT
    + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> u_raw_valid = u_raw.cwiseProduct(u_fusion_vec);

  // count the number of valid IMU signals
  for (int i = 0; i < TConfig::INPUT_VECTOR_SIZE * (TConfig::NUM_IMU_MEASUREMENT
    + TConfig::NUM_BACKUP_IMU_MEASUREMENT); ++i) {
    if (u_fusion_vec[i] > 0.0) {
      num_valid_imus[i % TConfig::INPUT_VECTOR_SIZE] += 1.0;
      u[i % TConfig::INPUT_VECTOR_SIZE] += u_raw_valid[i];
    }
  }

  // take the average of the IMU measurements if a valid measurement was received
  for (int i = 0; i < TConfig::INPUT_VECTOR_SIZE; ++i) {
    if (num_valid_imus[i] > 0.0) {
      u[i] /= num_valid_imus[i];
    }
  }

  return u;
}

template <class TConfig> const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>
  IMUHandler<TConfig>::filter_imu_measurements(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u_raw,
    const double imu_num)
{
  if (initialize_filter_) {
    for (int i = 0; i < TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT; ++i) {
      // set the Filter Coefficients
      Eigen::VectorXd imu_filter_coefficients_imu = Eigen::Map<Eigen::VectorXd>(
        param_manager_->get_parameter_value("P_VDC_IMU" + std::to_string(i + 1)
          + "_filter_coefficients").as_double_array().data(),
        param_manager_->get_parameter_value("P_VDC_IMU" + std::to_string(i + 1)
          + "_filter_coefficients").as_double_array().size());

      for (int vec_pos = 0; vec_pos < TConfig::INPUT_VECTOR_SIZE; ++vec_pos) {
        filter_imu_[i * TConfig::INPUT_VECTOR_SIZE + vec_pos] =
          std::make_unique<tam::core::state::FIR>(imu_filter_coefficients_imu);
      }
    }
    initialize_filter_ = false;
  }

  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u_filt;
  u_filt.setZero();
  // filter all values
  for (int i = 0; i < TConfig::INPUT_VECTOR_SIZE; ++i) {
    u_filt[i] = filter_imu_[imu_num * TConfig::INPUT_VECTOR_SIZE + i]->step(u_raw[i]);
  }
  return u_filt;
}

template <class TConfig> void
  IMUHandler<TConfig>::set_road_angles(
    const tam::types::common::Vector3D<double> & road_angles)
{
  // set the road angles
  road_angles_ = road_angles;
}

template <class TConfig> void
  IMUHandler<TConfig>::set_sensor_bias(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & imu_bias)
{
  // set the imu_bias
  imu_bias_ = imu_bias;
}

template <class TConfig> std::shared_ptr<tam::interfaces::ParamManagerBase>
  IMUHandler<TConfig>::get_param_handler(void)
{
  return param_manager_;
}
}  // namespace tam::core::state
