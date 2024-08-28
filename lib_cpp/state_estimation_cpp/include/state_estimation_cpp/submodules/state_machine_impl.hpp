// Copyright 2023 Marcel Weinmann
#pragma once
#include "state_estimation_cpp/submodules/state_machine.hpp"

template <class TConfig> tam::core::state::StateMachine<TConfig>::StateMachine()
{
  // param_manager
  param_manager_ = std::make_shared<tam::core::ParamManager>();

  // Minimum number of valid IMUs not to perform a safe stop
  param_manager_->declare_parameter(
    "P_VDC_MinValidIMUs", 2, tam::types::param::ParameterType::INTEGER, "");

  // initialize the valid map for better readability
  // valid bits for the position inputs
  for (int i = 1; i <= TConfig::NUM_POS_MEASUREMENT; ++i) {
    std::string key = "valid_pos_" + std::to_string(i);
    valid_map_[key] = false;
  }

  // valid bits for the position inputs
  for (int i = 1; i <= TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
    std::string key = "valid_orientation_" + std::to_string(i);
    valid_map_[key] = false;
  }

  // valid bits for the velocity inputs
  for (int i = 1; i <= TConfig::NUM_VEL_MEASUREMENT; ++i) {
    std::string key = "valid_lin_vel_" + std::to_string(i);
    valid_map_[key] = false;
  }

  // valid bits for the imu inputs
  for (int i = 1; i <= TConfig::NUM_IMU_MEASUREMENT; ++i) {
    std::string key = "valid_imu_" + std::to_string(i);
    valid_map_[key] = false;
  }

  valid_map_["backup_imu"] = backup_imu_active_;
}

/**
 * @brief Update the State Estimaion State Machine
 * 
 * OK:        State Estimation functions as expected
 * WARN:      The qualitiy of the state estimation prediction is degraded (no emergency)
 * ERROR:     An important sensor signal is missing (execute a soft emergency stop on EKF)
 * STALE:     All safety critical sensors are missing (execute a hard emergency stop)
 * 
 *                        |  OK  | STALE |  WARN  | STALE | ERROR | STALE | ERROR | STALE
 * ---------------------------------------------------------------------------------------
 * any valid_loc_x        |   x  |   x   |    x   |   x   |       |       |       |
 * ---------------------------------------------------------------------------------------
 * any valid_lin_vel_x    |   x  |   x   |        |       |   x   |   x   |       |
 * ---------------------------------------------------------------------------------------
 * any valid_imu_x        |   x  |       |    x   |       |   x   |       |   x   |
 * 
 * In addition to the truth table we perform a safe stop if:
 * number of valid imus < P_VDC_MinValidIMUs
 * 
 */
template <class TConfig> void tam::core::state::StateMachine<TConfig>::update(
  bool overwrite_state_machine)
{
  // check all valid whether atleast one input source per modalitiy is valid
  bool valid_loc = false, valid_lin_vel = false, valid_imu = false;
  bool valid_pos = false, valid_orientation = false, imu_safe_stop = false;

  // check whether there is a valid position input
  for (int i = 1; i <= TConfig::NUM_POS_MEASUREMENT; ++i) {
    std::string key = "valid_pos_" + std::to_string(i);
    valid_pos |= valid_map_[key];
  }

  // check whether there is a valid orientation input
  // dont consider the last two measurements because they are reserved for internal usage
  for (int i = 1; i <= TConfig::NUM_ORIENTATION_MEASUREMENT - 2; ++i) {
    std::string key = "valid_pos_" + std::to_string(i);
    valid_orientation |= valid_map_[key];
  }

  // combine the valid bits for position and orientation to obtain the a valid localization
  valid_loc = valid_orientation & valid_pos;

  // check whether there is a valid linear velocity input
  for (int i = 1; i <= TConfig::NUM_VEL_MEASUREMENT; ++i) {
    std::string key = "valid_lin_vel_" + std::to_string(i);
    valid_lin_vel |= valid_map_[key];
  }

  // check whether there is a valid imu input
  for (int i = 1; i <= (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT); ++i) {
    std::string key = "valid_imu_" + std::to_string(i);
    valid_imu |= valid_map_[key];
  }

  // perform a safe stop if the P_VDC_MinValidIMUs threshold is voilated or the backup imu mode
  // is activated (only active if one lin_vel and loc is valid)
  std::string backup_imu_key = "valid_imu_"
    + std::to_string(TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT);
  if (get_num_valid_imus() < param_manager_->get_parameter_value("P_VDC_MinValidIMUs").as_int()
      || (get_num_valid_imus() == 0 && backup_imu_active_ && valid_map_[backup_imu_key])) {
    imu_safe_stop = true;
  }

  // overwrite_state_machine allows any state transition
  // this should only be used to initialize or recover the state machine
  if (overwrite_state_machine) {
    if (valid_loc & valid_imu & valid_lin_vel & !imu_safe_stop) {
      state_ = tam::types::ErrorLvl::OK;
      status_message_ = "OK";
    } else if (((!imu_safe_stop) | (imu_safe_stop & valid_lin_vel & valid_loc)) & valid_imu) {
      state_ = tam::types::ErrorLvl::ERROR;
      if (!valid_loc) {
        status_message_ = "no valid loc";
      } else if (!valid_lin_vel) {
        status_message_ = "no valid lin_vel";
      } else {
        status_message_ = "imu safe stop";
      }
    } else {
      state_ = tam::types::ErrorLvl::STALE;
      status_message_ = "no valid imu";
    }
  } else {
    // set the state estimation status
    // if the state estimation status is OK or WARN all states can be reached
    if (state_ == tam::types::ErrorLvl::OK || state_ == tam::types::ErrorLvl::WARN) {
      if (valid_loc & valid_imu & valid_lin_vel & !imu_safe_stop) {
        state_ = tam::types::ErrorLvl::OK;
        status_message_ = "OK";
      } else if (((!imu_safe_stop) | (imu_safe_stop & valid_lin_vel & valid_loc)) & valid_imu) {
        state_ = tam::types::ErrorLvl::ERROR;
        if (!valid_loc) {
          status_message_ = "no valid loc";
        } else if (!valid_lin_vel) {
          status_message_ = "no valid lin_vel";
        } else {
          status_message_ = "imu safe stop";
        }
      } else {
        state_ = tam::types::ErrorLvl::STALE;
        status_message_ = "no valid imu";
      }
    // if the state estimation is in state Error the Error
    // is buffered and the state can only get worest
    } else if (state_ == tam::types::ErrorLvl::ERROR) {
      if (((!imu_safe_stop) | (imu_safe_stop & valid_lin_vel & valid_loc)) & valid_imu) {
        state_ = tam::types::ErrorLvl::ERROR;
        if (!valid_loc) {
          status_message_ = "no valid loc";
        } else if (imu_safe_stop) {
          status_message_ = "imu safe stop";
        } else if (!valid_lin_vel) {
          status_message_ = "no valid lin_vel";
        } else {
          status_message_ = "error buffered";
        }
      } else {
        state_ = tam::types::ErrorLvl::STALE;
        status_message_ = "no valid imu";
      }
    // if the state estimation is already in STALE it can only stay in STALE
    } else {
      state_ = tam::types::ErrorLvl::STALE;
        if (!valid_imu) {
          status_message_ = "no valid imu";
        } else {
          status_message_ = "stale buffered";
        }
    }
  }
}

/**
 * @brief forwards the status signal of the position input to the state estimation state machine
 *
 * @param[in] status            - tam::types::ErrorLvl:
 *                                containing information on the status of the position input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 * @param[in] pos_num           - uint8_t:
 *                                containing the number of the position input [0-N]
 */
template <class TConfig> void tam::core::state::StateMachine<TConfig>::set_position_status(
  const tam::types::ErrorLvl & status, uint8_t pos_num)
{
  // set the position input valid if the sensor status is OK (0)
  if (pos_num < TConfig::NUM_POS_MEASUREMENT) {
    std::string key = "valid_pos_" + std::to_string(pos_num + 1);
    if (status == tam::types::ErrorLvl::OK || status == tam::types::ErrorLvl::WARN) {
      valid_map_[key] = true;
    } else {
      valid_map_[key] = false;
    }
  }

  // update the state machine
  update();
}

/**
 * @brief forwards the status signal of the orientation input to the state estimation state machine
 *
 * @param[in] status                    - tam::types::ErrorLvl:
 *                                        containing information on the status of the orientation input
 *                                        (0: OK, 1: Warn, 2: Error, 3: STALE)
 * @param[in] orientation_num           - uint8_t:
 *                                        containing the number of the orientation input [0-N]
 */
template <class TConfig> void tam::core::state::StateMachine<TConfig>::set_orientation_status(
  const tam::types::ErrorLvl & status, uint8_t orientation_num)
{
  // set the position input valid if the sensor status is OK (0)
  if (orientation_num < TConfig::NUM_ORIENTATION_MEASUREMENT) {
    std::string key = "valid_orientation_" + std::to_string(orientation_num + 1);
    if (status == tam::types::ErrorLvl::OK || status == tam::types::ErrorLvl::WARN) {
      valid_map_[key] = true;
    } else {
      valid_map_[key] = false;
    }
  }

  // update the state machine
  update();
}

/**
 * @brief forwards the status signal of the linear velocity input to the state estimation state machine
 *
 * @param[in] status            - tam::types::ErrorLvl:
 *                                containing information on the status of the linear velocity input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 * @param[in] vel_num           - uint8_t:
 *                                containing the number of the linear velocity input [0-N]
 */
template <class TConfig> void tam::core::state::StateMachine<TConfig>::set_linear_velocity_status(
  const tam::types::ErrorLvl & status, uint8_t vel_num)
{
  // set the linear velocity input valid if the sensor status is OK (0)
  if (vel_num < TConfig::NUM_VEL_MEASUREMENT) {
    std::string key = "valid_lin_vel_" + std::to_string(vel_num + 1);
    if (status == tam::types::ErrorLvl::OK) {
      valid_map_[key] = true;
    } else {
      valid_map_[key] = false;
    }
  }

  // update the state machine
  update();
}

/**
 * @brief forwards the status signal of the imu input to the state estimation state machine
 *
 * @param[in] status            - tam::types::ErrorLvl:
 *                                containing information on the status of the imu input
 *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
 * @param[in] imu_num           - uint8_t:
 *                                containing the number of the imu input [0-N]
 */
template <class TConfig> void tam::core::state::StateMachine<TConfig>::set_imu_status(
  const tam::types::ErrorLvl & status, uint8_t imu_num)
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

/**
 * @brief sets the position input in the state estimation state machine invalid
 *        (signal is not longer fused, and state machine is updated)
 *
 * @param[in] pos_num           - uint8_t:
 *                                containing the number of the position input [0-N]
 */
template <class TConfig> void tam::core::state::StateMachine<TConfig>::set_position_invalid(
  uint8_t pos_num)
{
  // set the position input invalid
  if (pos_num < TConfig::NUM_POS_MEASUREMENT) {
    std::string key = "valid_pos_" + std::to_string(pos_num + 1);
    valid_map_[key] = false;
  }

  // update the state machine
  update();
}

/**
 * @brief sets the orientation input in the state estimation state machine invalid
 *        (signal is not longer fused, and state machine is updated)
 *
 * @param[in] pos_num           - uint8_t:
 *                                containing the number of the position input [0-N]
 */
template <class TConfig> void tam::core::state::StateMachine<TConfig>::set_orientation_invalid(
  uint8_t orientation_num)
{
  // set the position input invalid
  if (orientation_num < TConfig::NUM_ORIENTATION_MEASUREMENT) {
    std::string key = "valid_orientation_" + std::to_string(orientation_num + 1);
    valid_map_[key] = false;
  }

  // update the state machine
  update();
}

/**
 * @brief sets the linear velocity input in the state estimation state machine invalid
 *        (signal is not longer fused, and state machine is updated)
 *
 * @param[in] vel_num           - uint8_t:
 *                                containing the number of the linear velocity input [0-N]
 */
template <class TConfig> void tam::core::state::StateMachine<TConfig>::set_linear_velocity_invalid(
  uint8_t vel_num)
{
  // set the linear velocity input invalid
  if (vel_num < TConfig::NUM_VEL_MEASUREMENT) {
    std::string key = "valid_lin_vel_" + std::to_string(vel_num + 1);
    valid_map_[key] = false;
  }

  // update the state machine
  update();
}

/**
 * @brief sets the imu input in the state estimation state machine invalid
 *        (signal is not longer fused, and state machine is updated)
 *
 * @param[in] imu_num           - uint8_t:
 *                                containing the number of the imu input [0-N]
 */
template <class TConfig> void tam::core::state::StateMachine<TConfig>::set_imu_invalid(
  uint8_t imu_num)
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

/**
 * @brief sets the backup imu of the car active to perform a safe stop
 */
template <class TConfig> void tam::core::state::StateMachine<TConfig>::set_backup_imu_active(void)
{
  // active the backup imu to perform a safe stop on this one imu if it is valid
  int backup_imu_num = TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT;
  std::string key = "valid_imu_" + std::to_string(backup_imu_num);
  if (valid_map_[key]) {
    backup_imu_active_ = true;
    valid_map_["backup_imu"] = true;
  }

  // update the state machine
  update();
}

/**
 * @brief sets the backup imu of the car inactive
 */
template <class TConfig> void tam::core::state::StateMachine<TConfig>::set_backup_imu_inactive(void)
{
  // deactivate the backup imu
  backup_imu_active_ = false;
  valid_map_["backup_imu"] = false;

  // update the state machine
  update();
}

/**
 * @brief Get the current State of the State Machine
 *
 * @param[out]                  - tam::types::ErrorLvl:
 *                                current state of the state estimation
 */
template <class TConfig> tam::types::ErrorLvl tam::core::state::StateMachine<TConfig>::get_state(
  void)
{
  return state_;
}

/**
 * @brief Get a vector of the size of the measurement vector indicating which signal is valid
 *
 * @param[out]                  - Eigen::VectorXd:
 *                                fusion vector which signal should be fused
 */
template <class TConfig> Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>
  tam::core::state::StateMachine<TConfig>::get_valid_fusion_vec(void)
{
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> valid_fusion_vec;
  valid_fusion_vec.setZero();

  // check all position valid bits
  for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
    std::string key = "valid_pos_" + std::to_string(i + 1);
    if (valid_map_[key]) {
      valid_fusion_vec.segment(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE,
                               TConfig::POS_MEASUREMENT_VECTOR_SIZE)
        = Eigen::VectorXd::Ones(TConfig::POS_MEASUREMENT_VECTOR_SIZE);
    }
  }

  // check all orientation valid bits
  for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
    std::string key = "valid_orientation_" + std::to_string(i + 1);
    if (valid_map_[key]) {
      valid_fusion_vec.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION
                               + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE,
                               TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE)
        = Eigen::VectorXd::Ones(TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE);
    }
  }

  // check linear velocity valid bits
  for (int i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
    std::string key = "valid_lin_vel_" + std::to_string(i + 1);
    if (valid_map_[key]) {
      valid_fusion_vec.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_VEL
                               + i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE,
                               TConfig::VEL_MEASUREMENT_VECTOR_SIZE)
        = Eigen::VectorXd::Ones(TConfig::VEL_MEASUREMENT_VECTOR_SIZE);
    }
  }

  return valid_fusion_vec;
}

/**
 * @brief Get a vector of the size of the raw u (imu input) vector indicating which signal is valid
 * 
 * @param[out]                  - Eigen::VectorXd:
 *                                fusion vector which signal should be fused
 */
template <class TConfig>
const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE
  * (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>
  tam::core::state::StateMachine<TConfig>::get_valid_u_fusion_vec(void)
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

/**
 * @brief Returns the number of currently valid imus
 *
 * @param[out]                          - uint8_t:
 *                                        number of valid imus
 */
template <class TConfig> uint8_t tam::core::state::StateMachine<TConfig>::get_num_valid_imus(void){
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

/**
 * @brief Resturns a string containing the current status message of the state
 *
 * @param[out]                          - std::string
 */
template <class TConfig> std::string tam::core::state::StateMachine<TConfig>::get_status_msg(void){
  return status_message_;
}

/**
 * @brief get all individual valid bits for the state estimation output
 *
 * @param[out]                  - std::map<std::string, bool> (valid_map_):
 *                                entire valid_map_ containing the sensor valid bits
 */
template <class TConfig>
std::map<std::string, bool> tam::core::state::StateMachine<TConfig>::get_debug(void)
{
  return valid_map_;
}

/**
 * @brief returns a pointer to the param manager
 *
 * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
 */
template <class TConfig> std::shared_ptr<tam::interfaces::ParamManagerBase>
  tam::core::state::StateMachine<TConfig>::get_param_handler(void)
{
  return param_manager_;
}
