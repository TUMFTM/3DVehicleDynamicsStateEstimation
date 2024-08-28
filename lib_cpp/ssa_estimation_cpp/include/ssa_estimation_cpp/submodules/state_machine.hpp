// Copyright 2024 Sven Goblirsch
#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// type definitions
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// Param manager
#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager.hpp"

// SSA Estimation constants / template input
#include "ssa_estimation_constants/UKF_STM.hpp"

namespace tam::core::ssa
{
template <class TConfig>
class StateMachine
{
private:
  // Variables
  /**
   * @brief Status of the Side Slip Angle Estimation
   *        (0: OK, 1: WARN, 2: ERROR, 3: STALE)
   */
  tam::types::ErrorLvl state_{tam::types::ErrorLvl::ERROR};

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
  bool backup_imu_active_{false};

  /**
   * @brief Indicate whether kalman filter output is physically feasable or filter diverged
   */
  bool feasable_output_{false};

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
   * @brief Update the State Machine
   *
   */
  void update();

  /**
   * @brief forwards the status signal of the imu input to the state machine
   *
   * @param[in] status                    - tam::types::ErrorLvl:
   *                                        containing information on the status of the localization
   * input (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num                   - uint8_t:
   *                                        containing the number of the imu input [0-N]
   */
  void set_imu_status(const tam::types::ErrorLvl & status, const uint8_t imu_num);

  /**
   * @brief sets the imu input in the ssa estimation state machine invalid
   *        (signal is not longer fused, and state machine is updated)
   *
   * @param[in] imu_num                   - uint8_t:
   *                                        containing the number of the imu input [0-N]
   */
  void set_imu_invalid(const uint8_t imu_num);

  /**
   * @brief sets the backup imu of the car active to perform a safe stop
   */
  void set_backup_imu_active();

  /**
   * @brief sets the backup imu of the car inactive
   */
  void set_backup_imu_inactive();

  /**
  * @brief sets the steering status in the state machine
  * 
  * @param[in] status            - tam::types::ErrorLvl:
  *                                containing information on the status of the sensor input
  *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
  */
  void set_steering_status(const tam::types::ErrorLvl & status);

  /**
  * @brief sets the wheelspeed status in the state machine
  * 
  * @param[in] status            - tam::types::ErrorLvl:
  *                                containing information on the status of the sensor input
  *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
  */
  void set_wheelspeed_status(const tam::types::ErrorLvl & status);

  /**
  * @brief sets the drivetrain torque status in the state machine
  * 
  * @param[in] status            - tam::types::ErrorLvl:
  *                                containing information on the status of the sensor input
  *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
  */
  void set_drivetrain_torque_status(const tam::types::ErrorLvl & status);

  /**
  * @brief sets the brake pressure status in the state machine
  * 
  * @param[in] status            - tam::types::ErrorLvl:
  *                                containing information on the status of the sensor input
  *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
  */
  void set_brake_pressure_status(const tam::types::ErrorLvl & status);

  /**
  * @brief sets the orientation status in the state machine
  * 
  * @param[in] status            - tam::types::ErrorLvl:
  *                                containing information on the status of the sensor input
  *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
  */
  void set_orientation_status(const tam::types::ErrorLvl & status);

  /**
   * @brief checks the physical feasability of the output vector
   * 
   * @param[in] x                 - Vector containing kalman state
   * 
   */
  void check_output_feasability(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & x);

  /**
   * @brief Get the current State of the State Machine
   *
   * @param[out]                          - tam::types::ErrorLvl:
   *                                        current state of the side slip angle estimation
   */
  tam::types::ErrorLvl get_state();

  /**
   * @brief Get a vector of the size of the raw u (imu input) vector indicating which signal is valid
   *
   * @param[out]                          - Eigen::VectorXd:
   *                                        fusion vector which signal should be fused
   */
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE * (TConfig::NUM_IMU_MEASUREMENT
    + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> get_valid_u_fusion_vec();

  /**
   * @brief Resturns the number of currently valid imus
   *
   * @param[out]                          - uint8_t:
   *                                        number of valid imus
   */
  uint8_t get_num_valid_imus();

  /**
   * @brief Resturns a string containing the current status message of the state
   *
   * @param[out]                          - std::string
   */
  std::string get_status_msg();

  /**
   * @brief get all individual valid bits for the ssa estimation output
   *
   * @param[out]                          - std::map<std::string, bool> (valid_map_):
   *                                        entire valid_map_ containing the sensor valid bits
   */
  std::map<std::string, bool> get_debug();

  /**
   * @brief returns a pointer to the param manager
   *
   * @param[out]                          - std::shared_ptr<tam::interfaces::ParamManagerBase>
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler();
};
}  // namespace tam::core::ssa
# include "ssa_estimation_cpp/submodules/state_machine_impl.hpp"
