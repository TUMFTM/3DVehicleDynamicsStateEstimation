// Copyright 2023 Marcel Weinmann
#pragma once
#include <vector>
#include <string>

namespace tam::core::state
{
struct KF_BASE
{
  /**
   * @brief State Estimation constants
   */
  static inline double TS = 0.01;
  static inline constexpr int NUM_IMU_MEASUREMENT = 3;
  static inline constexpr int NUM_BACKUP_IMU_MEASUREMENT = 1;

  /**
   * @brief Kalman filter base constants (number inputs)
   *        NUM_ORIENTATION_MEASUREMENT should always be N + 2 due to internal variable handling
   */
  static inline constexpr int NUM_POS_MEASUREMENT = 3;
  static inline constexpr int NUM_ORIENTATION_MEASUREMENT = 5;
  static inline constexpr int NUM_VEL_MEASUREMENT = 3;
};
}  // namespace tam::core::state
