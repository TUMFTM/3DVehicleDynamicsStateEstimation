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

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

namespace tam::core::state
{
template <class TConfig>
class StateMachine
{
private:
  // Variables
  /**
   * @brief Status of the State Estimation
   *        (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  tam::types::ErrorLvl state_;

  /**
   * @brief Vector containing all the sensor biases for the imu signals
   */
  std::map<std::string, bool> valid_map_;

  /**
   * @brief Status message of the state machine describing the current state
   */
  std::string status_message_;

  /**
   * @brief Indicate whether the backup imu is active
   */
  bool backup_imu_active_ = false;

  /**
   * @brief IAC Parameter Manager
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> param_manager_;

public:
  /**
   * @brief Constructor
   */
  StateMachine();

  /**
   * @brief Update the State Estimaion State Machine
   *
   * @param[in] overwrite_state_machine   - bool:
   *                                        this function input allows the overwrite the state machine logic 
   *                                        such that all states can be reached.
   */
  void update(bool overwrite_state_machine = false);

  /**
   * @brief forwards the status signal of the position input to the state estimation state machine
   *
   * @param[in] status                    - tam::types::ErrorLvl:
   *                                        containing information on the status of the position input
   *                                        (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] pos_num                   - uint8_t:
   *                                        containing the number of the position input [0-N]
   */
  void set_position_status(const tam::types::ErrorLvl & status, uint8_t pos_num);

  /**
   * @brief forwards the status signal of the orientation input to the state estimation state machine
   *
   * @param[in] status                    - tam::types::ErrorLvl:
   *                                        containing information on the status of the orientation input
   *                                        (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] orientation_num           - uint8_t:
   *                                        containing the number of the orientation input [0-N]
   */
  void set_orientation_status(const tam::types::ErrorLvl & status, uint8_t orientation_num);

  /**
   * @brief forwards the status signal of the linear velocity input to the state estimation state machine
   *
   * @param[in] status                    - tam::types::ErrorLvl:
   *                                        containing information on the status of the localization input
   *                                        (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] vel_num                   - uint8_t:
   *                                        containing the number of the linear velocity input [0-N]
   */
  void set_linear_velocity_status(const tam::types::ErrorLvl & status, uint8_t vel_num);

  /**
   * @brief forwards the status signal of the imu input to the state estimation state machine
   *
   * @param[in] status                    - tam::types::ErrorLvl:
   *                                        containing information on the status of the localization input
   *                                        (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num                   - uint8_t:
   *                                        containing the number of the imu input [0-N]
   */
  void set_imu_status(const tam::types::ErrorLvl & status, uint8_t imu_num);

  /**
   * @brief sets the position input in the state estimation state machine invalid
   *        (signal is not longer fused, and state machine is updated)
   *
   * @param[in] pos_num                   - uint8_t:
   *                                        containing the number of the position input [0-N]
   */
  void set_position_invalid(uint8_t pos_num);

  /**
   * @brief sets the orientation input in the state estimation state machine invalid
   *        (signal is not longer fused, and state machine is updated)
   *
   * @param[in] orientation_num           - uint8_t:
   *                                        containing the number of the orientation input [0-N]
   */
  void set_orientation_invalid(uint8_t orientation_num);

  /**
   * @brief sets the linear velocity input in the state estimation state machine invalid
   *        (signal is not longer fused, and state machine is updated)
   *
   * @param[in] vel_num                   - uint8_t:
   *                                        containing the number of the linear velocity input [0-N]
   */
  void set_linear_velocity_invalid(uint8_t vel_num);

  /**
   * @brief sets the imu input in the state estimation state machine invalid
   *        (signal is not longer fused, and state machine is updated)
   *
   * @param[in] imu_num                   - uint8_t:
   *                                        containing the number of the imu input [0-N]
   */
  void set_imu_invalid(uint8_t imu_num);

  /**
   * @brief sets the backup imu of the car active to perform a safe stop
   */
  void set_backup_imu_active(void);

  /**
   * @brief sets the backup imu of the car inactive
   */
  void set_backup_imu_inactive(void);

  /**
   * @brief Get the current State of the State Machine
   *
   * @param[out]                          - tam::types::ErrorLvl:
   *                                        current state of the state estimation
   */
  tam::types::ErrorLvl get_state(void);

  /**
   * @brief Get a vector of the size of the measurement vector indicating which signal is valid
   *
   * @param[out]                          - Eigen::VectorXd:
   *                                        fusion vector which signal should be fused
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> get_valid_fusion_vec(void);

  /**
   * @brief Get a vector of the size of the raw u (imu input) vector indicating which signal is valid
   *
   * @param[out]                          - Eigen::VectorXd:
   *                                        fusion vector which signal should be fused
   */
  const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE * (TConfig::NUM_IMU_MEASUREMENT
    + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> get_valid_u_fusion_vec(void);

  /**
   * @brief Resturns the number of currently valid imus
   *
   * @param[out]                          - uint8_t:
   *                                        number of valid imus
   */
  uint8_t get_num_valid_imus(void);

  /**
   * @brief Resturns a string containing the current status message of the state
   *
   * @param[out]                          - std::string
   */
  std::string get_status_msg(void);

  /**
   * @brief get all individual valid bits for the state estimation output
   *
   * @param[out]                          - std::map<std::string, bool> (valid_map_):
   *                                        entire valid_map_ containing the sensor valid bits
   */
  std::map<std::string, bool> get_debug(void);

  /**
   * @brief returns a pointer to the param manager
   *
   * @param[out]                          - std::shared_ptr<tam::interfaces::ParamManagerBase>
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler(void);
};
}  // namespace tam::core::state
#include "state_estimation_cpp/submodules/state_machine_impl.hpp"
