// Copyright 2024 Sven Goblirsch
#pragma once
#include <string>
#include <vector>

namespace tam::core::ssa
{
struct UKF_STM
{
  /**
   * @brief SSA Estimation constants
   */
  static inline double TS = 0.01;
  static inline constexpr int NUM_IMU_MEASUREMENT = 3;
  static inline constexpr int NUM_BACKUP_IMU_MEASUREMENT = 1;

  /**
   * @brief Size of the state vector
   */
  enum { STATE_VEL_MPS, STATE_BETA_RAD, STATE_VECTOR_SIZE };

  /**
   * @brief Size of the imu input vector
   */
  enum {
    INPUT_DPHI_RADPS,
    INPUT_DTHETA_RADPS,
    INPUT_DPSI_RADPS,
    INPUT_AX_MPS2,
    INPUT_AY_MPS2,
    INPUT_AZ_MPS2,
    INPUT_VECTOR_SIZE
  };

  /**
   * @brief Kalman filter base constants: Size of the measurement vector
   */
  enum {
    MEASUREMENT_DELTA_RAD,
    MEASUREMENT_OMEGA_WHEEL_FL_RADPS,
    MEASUREMENT_OMEGA_WHEEL_FR_RADPS,
    MEASUREMENT_OMEGA_WHEEL_RL_RADPS,
    MEASUREMENT_OMEGA_WHEEL_RR_RADPS,
    MEASUREMENT_YAW_ACC_RADPS2,
    MEASUREMENT_DRIVETRAIN_TORQUE_NM,
    MEASUREMENT_BRAKE_PRESSURE_FL_PA,
    MEASUREMENT_BRAKE_PRESSURE_FR_PA,
    MEASUREMENT_BRAKE_PRESSURE_RL_PA,
    MEASUREMENT_BRAKE_PRESSURE_RR_PA,
    MEASUREMENT_ROAD_ANGLES_PITCH_RAD,
    MEASUREMENT_ROAD_ANGLES_ROLL_RAD,
    MEASUREMENT_VECTOR_SIZE
    };

    /**
   * @brief Kalman filter base constants: vector containing all inputs for the kalman prediction
   */
  enum {
    PROCESS_DELTA_RAD,
    PROCESS_OMEGA_WHEEL_FL_RADPS,
    PROCESS_OMEGA_WHEEL_FR_RADPS,
    PROCESS_OMEGA_WHEEL_RL_RADPS,
    PROCESS_OMEGA_WHEEL_RR_RADPS,
    PROCESS_DPSI_RADPS,
    PROCESS_AX_MPS2,
    PROCESS_AY_MPS2,
    PROCESS_AZ_MPS2,
    PROCESS_ROAD_ANGLES_PITCH_RAD,
    PROCESS_ROAD_ANGLES_ROLL_RAD,
    PROCESS_VECTOR_SIZE
    };

  /**
   * @brief Kalman filter base constants: vector containing all inputs for the kalman update
   */
  enum {
    UPDATE_DELTA_RAD,
    UPDATE_OMEGA_WHEEL_FL_RADPS,
    UPDATE_OMEGA_WHEEL_FR_RADPS,
    UPDATE_OMEGA_WHEEL_RL_RADPS,
    UPDATE_OMEGA_WHEEL_RR_RADPS,
    UPDATE_DPSI_RADPS,
    UPDATE_AX_MPS2,
    UPDATE_AY_MPS2,
    UPDATE_AZ_MPS2,
    UPDATE_YAW_ACC_RADPS2,
    UPDATE_DRIVETRAIN_TORQUE_NM,
    UPDATE_BRAKE_PRESSURE_FL_PA,
    UPDATE_BRAKE_PRESSURE_FR_PA,
    UPDATE_BRAKE_PRESSURE_RL_PA,
    UPDATE_BRAKE_PRESSURE_RR_PA,
    UPDATE_ROAD_ANGLES_PITCH_RAD,
    UPDATE_ROAD_ANGLES_ROLL_RAD,
    UPDATE_VECTOR_SIZE
    };

  /**
   * @brief Kalman filter base constants: Size of the virtual measurement vector
   */
  enum {
    VIRTMEAS_V_STM_1_MPS,
    VIRTMEAS_V_STM_2_MPS,
    VIRTMEAS_FTX_F_N,
    VIRTMEAS_FTX_R_N,
    VIRTMEAS_FTY_F_N,
    VIRTMEAS_FTY_R_N,
    VIRTMEAS_VECTOR_SIZE
  };

  /**
   * @brief Enum containing the position of the wheel
   */
  enum { FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT };
  };
}  // namespace tam::core::ssa
