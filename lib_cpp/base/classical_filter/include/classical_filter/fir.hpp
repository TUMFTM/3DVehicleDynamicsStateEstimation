// Copyright 2023 Marcel Weinmann
#pragma once

#include <deque>
#include <memory>
#include <eigen3/Eigen/Dense>

namespace tam::core::state
{
class FIR
{
private:
  // Variables
  /**
   * @brief filter coefficients of the FIR filter
   */
  Eigen::VectorXd filter_coefficients_;

  /**
   * @brief filter coefficients of the FIR filter
   */
  std::deque<double> fifo_;

public:
  /**
   * @brief Constructor
   */
  explicit FIR(const Eigen::Ref<const Eigen::VectorXd> & coefficients);

  /**
   * @brief Step the FIR filter once
   *
   * @param[in] input         - double:
   *                            value to be inserted in the buffer
   * @param[out]              - double:
   *                            filtered value by the FIR filter
   */
  double step(double input);
};
}  // namespace tam::core::state
#include "classical_filter/fir_impl.hpp"
