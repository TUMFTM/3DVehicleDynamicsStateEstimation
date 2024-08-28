// Copyright 2024 Sven Goblirsch
#pragma once
#include "ssa_estimation_cpp/submodules/state_machine.hpp"

namespace tam::core::ssa
{
template <class TConfig> StateMachine<TConfig>::StateMachine()
{
  // param_manager
  param_manager_ = std::make_shared<tam::core::ParamManager>();

  // initialize the valid map
  // valid bits for the imu inputs
  for (int i = 1; i <= (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT); ++i) {
    std::string key = "valid_imu_" + std::to_string(i);
    valid_map_[key] = false;
  }
  valid_map_["valid_steering"] = false;
  valid_map_["valid_wheelspeed"] = false;
  valid_map_["valid_orientation"] = false;
  valid_map_["valid_drivetrain_torque"] = false;
  valid_map_["valid_brake_pressure"] = false;
}

template <class TConfig> void
  StateMachine<TConfig>::update()
{
  /**
  *                           |  OK  | WARN | ERROR |
  * -------------------------------------------------
  * any valid_imu_x           |   x  |  x   |       |
  * -------------------------------------------------
  * valid_steering            |   x  |  x   |       |
  * -------------------------------------------------
  * valid_wheelspeed          |   x  |  x   |       |
  * -------------------------------------------------
  * valid_drivetrain_torque   |   x  |      |       |
  * -------------------------------------------------
  * valid_brake_pressure      |   x  |      |       |
  * -------------------------------------------------
  * valid_orientation         |      |      |       |
  */

  // check all valid whether at least one input source per modalitiy is valid
  bool valid_imu = false;
  bool valid_backup_imu = false;
  bool valid_steering = valid_map_["valid_steering"];
  bool valid_wheelspeed = valid_map_["valid_wheelspeed"];
  bool valid_drivetrain_torque = valid_map_["valid_drivetrain_torque"];
  bool valid_brake_pressure = valid_map_["valid_brake_pressure"];
  bool valid_orientation = valid_map_["valid_orientation"];

  // check whether there is a valid imu input
  for (int i = 1; i <= TConfig::NUM_IMU_MEASUREMENT; ++i) {
    std::string key = "valid_imu_" + std::to_string(i);
    valid_imu |= valid_map_[key];
  }

  std::string backup_imu_key = "valid_imu_"
    + std::to_string(TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT);
  if (backup_imu_active_ && valid_map_[backup_imu_key]) { valid_backup_imu = true; }

  if (valid_imu & valid_steering & valid_wheelspeed & feasable_output_
    & valid_drivetrain_torque & valid_brake_pressure) {
    state_ = tam::types::ErrorLvl::OK;
    status_message_ = "OK";
    if (!valid_orientation) {
      status_message_ = "no valid orientation meas";
    }
  } else if ((valid_imu | valid_backup_imu) & valid_steering &
              valid_wheelspeed & feasable_output_) {
      state_ = tam::types::ErrorLvl::WARN;
      if (valid_imu & valid_drivetrain_torque & valid_brake_pressure) {
      } else if (valid_imu & valid_drivetrain_torque) {
        status_message_ = "no valid brake pressure meas";
      } else if (valid_imu) {
        status_message_ = "no valid drivetrain torque meas";
      } else {
        status_message_ = "only backup imu";
      }
  } else {
    state_ = tam::types::ErrorLvl::ERROR;
    if (!feasable_output_) {
      status_message_ = "output unfeasable";
    } else if (!valid_imu) {
      status_message_ = "no valid imu";
    } else if (!valid_steering) {
      status_message_ = "no valid steering meas";
    } else {
      status_message_ = "no valid wheelspeed meas";
    }
  }
}

template <class TConfig> void
  StateMachine<TConfig>::set_imu_status(
    const tam::types::ErrorLvl & status, const uint8_t imu_num)
{
  // set the imu input valid if the sensor status is OK (0)
  if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
    std::string key = "valid_imu_" + std::to_string(imu_num + 1);
    if (status == tam::types::ErrorLvl::OK) {
      valid_map_[key] = true;
      if (imu_num < TConfig::NUM_IMU_MEASUREMENT) {
        set_backup_imu_inactive();
      }
    } else {
      valid_map_[key] = false;
      // handle backup imu
      if (imu_num < TConfig::NUM_IMU_MEASUREMENT && get_num_valid_imus() == 1) {
        set_backup_imu_active();
      }
    }
  }
  // update the state machine
  update();
}

template <class TConfig> void
  StateMachine<TConfig>::set_imu_invalid(
    const uint8_t imu_num)
{
  // set the imu input invalid
  std::string key = "valid_imu_" + std::to_string(imu_num + 1);
  if (imu_num < TConfig::NUM_IMU_MEASUREMENT) {
    if (get_num_valid_imus() == 1) {
      set_backup_imu_active();
    }
    valid_map_[key] = false;
  } else if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
    valid_map_[key] = false;
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
  }
  // update the state machine
  update();
}

template <class TConfig> void
  StateMachine<TConfig>::set_backup_imu_active()
{
  // active the backup imu
  int backup_imu_num = TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT;
  std::string key = "valid_imu_" + std::to_string(backup_imu_num);
  if (valid_map_[key]) {
    backup_imu_active_ = true;
    valid_map_[key] = true;
  }
  // update the state machine
  update();
}

template <class TConfig> void
  StateMachine<TConfig>::set_backup_imu_inactive()
{
  // deactivate the backup imu
  int backup_imu_num = TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT;
  std::string key = "valid_imu_" + std::to_string(backup_imu_num);
  backup_imu_active_ = false;
  valid_map_[key] = false;
  // update the state machine
  update();
}

template <class TConfig> void
  StateMachine<TConfig>::set_steering_status(
    const tam::types::ErrorLvl & status)
{
  // set the steering input valid if the sensor status is OK (0)
  if (status == tam::types::ErrorLvl::OK) {
    valid_map_["valid_steering"] = true;
  } else {
    valid_map_["valid_steering"] = false;
  }
  // update the state machine
  update();
}

template <class TConfig> void
  StateMachine<TConfig>::set_wheelspeed_status(
    const tam::types::ErrorLvl & status)
{
  // set the wheelspeed input valid if the sensor status is OK (0)
  if (status == tam::types::ErrorLvl::OK) {
    valid_map_["valid_wheelspeed"] = true;
  } else {
    valid_map_["valid_wheelspeed"] = false;
  }
  // update the state machine
  update();
}

template <class TConfig> void
  StateMachine<TConfig>::set_drivetrain_torque_status(
    const tam::types::ErrorLvl & status)
{
  // set the drivetrain torque input valid if the sensor status is OK (0)
  if (status == tam::types::ErrorLvl::OK) {
    valid_map_["valid_drivetrain_torque"] = true;
  } else {
    valid_map_["valid_drivetrain_torque"] = false;
  }
  // update the state machine
  update();
}

template <class TConfig> void
  StateMachine<TConfig>::set_brake_pressure_status(
    const tam::types::ErrorLvl & status)
{
  // set the brake pressure input valid if the sensor status is OK (0)
  if (status == tam::types::ErrorLvl::OK) {
    valid_map_["valid_brake_pressure"] = true;
  } else {
    valid_map_["valid_brake_pressure"] = false;
  }
  // update the state machine
  update();
}

template <class TConfig> void
  StateMachine<TConfig>::set_orientation_status(
    const tam::types::ErrorLvl & status)
{
  // set the orientation input valid if the sensor status is OK (0)
  if (status == tam::types::ErrorLvl::OK) {
    valid_map_["valid_orientation"] = true;
  } else {
    valid_map_["valid_orientation"] = false;
  }
  // update the state machine
  update();
}

template <class TConfig> void
  StateMachine<TConfig>::check_output_feasability(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & x)
{
  if ((x[TConfig::STATE_VEL_MPS] <= 100.00) && (x[TConfig::STATE_VEL_MPS] >= -20.00)) {
    feasable_output_ = true;
  } else { feasable_output_ = false; }
}

template <class TConfig> tam::types::ErrorLvl
  StateMachine<TConfig>::get_state()
{
  return state_;
}

template <class TConfig> Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE
  * (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>
  StateMachine<TConfig>::get_valid_u_fusion_vec()
{
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE * (TConfig::NUM_IMU_MEASUREMENT
    + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> valid_u_fusion_vec;
  valid_u_fusion_vec.setZero();

  // check imu valid bits only use the standard imus if the backup imu mode is not set active
  int num_used_imu_measurement = TConfig::NUM_IMU_MEASUREMENT;
  if (backup_imu_active_) {
    num_used_imu_measurement += TConfig::NUM_BACKUP_IMU_MEASUREMENT;
  }

  for (int i = 0; i < num_used_imu_measurement; ++i) {
    std::string key = "valid_imu_" + std::to_string(i + 1);
    if (valid_map_[key]) {
      valid_u_fusion_vec.segment(i * TConfig::INPUT_VECTOR_SIZE, TConfig::INPUT_VECTOR_SIZE)
        = Eigen::VectorXd::Ones(TConfig::INPUT_VECTOR_SIZE);
    }
  }

  return valid_u_fusion_vec;
}

template <class TConfig> uint8_t
  StateMachine<TConfig>::get_num_valid_imus()
{
  uint8_t num_valid_imus = 0;

  // check how many imus are valid
  for (int i = 1; i <= TConfig::NUM_IMU_MEASUREMENT; ++i) {
    std::string key = "valid_imu_" + std::to_string(i);
    if (valid_map_[key]) {
      num_valid_imus += 1;
    }
  }

  return num_valid_imus;
}

template <class TConfig> std::string
  StateMachine<TConfig>::get_status_msg()
{
  return status_message_;
}

template <class TConfig> std::map<std::string, bool>
  StateMachine<TConfig>::get_debug()
{
  return valid_map_;
}

template <class TConfig> std::shared_ptr<tam::interfaces::ParamManagerBase>
  StateMachine<TConfig>::get_param_handler()
{
  return param_manager_;
}
}  // namespace tam::core::ssa