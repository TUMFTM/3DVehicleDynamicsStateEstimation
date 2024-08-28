// Copyright 2023 TUM
#pragma once

#include <math.h>

#include <eigen3/Eigen/Dense>

#include "tum_types_cpp/common.hpp"
namespace tam::helpers::euler_rotations
{
struct Eigen3D
{
  Eigen3D(const Eigen::VectorXd & x_in, const Eigen::VectorXd & y_in, const Eigen::VectorXd & z_in)
  {
    x = x_in;
    y = y_in;
    z = z_in;
  }
  Eigen3D() = default;
  Eigen::VectorXd x;
  Eigen::VectorXd y;
  Eigen::VectorXd z;
  void resize(int size)
  {
    x.resize(size);
    y.resize(size);
    z.resize(size);
  }
};
/**
 *
 * @brief Rotate input vector around Yaw, Pitch, Roll
 *        (e.g. local cartesian to vehicle frame)
 *
 * @param[in] input         - Input vector to be rotated
 *
 * @param[in] euler_ypr     - Euler angles
 */
namespace math
{
inline tam::types::common::Vector3D<double> rotate_ypr(
  const tam::types::common::Vector3D<double> & input,
  const tam::types::common::EulerYPR & euler_ypr)
{
  tam::types::common::Vector3D<double> output;

  double sphi = std::sin(euler_ypr.roll), cphi = std::cos(euler_ypr.roll);
  double stheta = std::sin(euler_ypr.pitch), ctheta = std::cos(euler_ypr.pitch);
  double spsi = std::sin(euler_ypr.yaw), cpsi = std::cos(euler_ypr.yaw);

  output.x = ctheta * cpsi * input.x + (sphi * stheta * cpsi - cphi * spsi) * input.y +
             (cphi * stheta * cpsi + sphi * spsi) * input.z;
  output.y = ctheta * spsi * input.x + (sphi * stheta * spsi + cphi * cpsi) * input.y +
             (cphi * stheta * spsi - sphi * cpsi) * input.z;
  output.z = -stheta * input.x + sphi * ctheta * input.y + cphi * ctheta * input.z;

  return output;
}
/**
 * @brief Rotate input vector around Yaw, Pitch, Roll
 *        (e.g. local cartesian to vehicle frame)
 *
 * @param[in] input         - Input vector to be rotated
 *                            idx: 0 - x, 1 - y, 2 - z
 *
 * @param[in] euler_ypr     - Euler angles
 *                            idx: 0 - roll, 1 - pitch, 2 - yaw
 */
inline const Eigen::Vector3d rotate_ypr(
  const Eigen::Ref<const Eigen::Vector3d> & input,
  const Eigen::Ref<const Eigen::Vector3d> & euler_ypr)
{
  Eigen::Vector3d output;

  double sphi = std::sin(euler_ypr[0]), cphi = std::cos(euler_ypr[0]);
  double stheta = std::sin(euler_ypr[1]), ctheta = std::cos(euler_ypr[1]);
  double spsi = std::sin(euler_ypr[2]), cpsi = std::cos(euler_ypr[2]);

  output[0] = ctheta * cpsi * input[0] + (sphi * stheta * cpsi - cphi * spsi) * input[1] +
              (cphi * stheta * cpsi + sphi * spsi) * input[2];
  output[1] = ctheta * spsi * input[0] + (sphi * stheta * spsi + cphi * cpsi) * input[1] +
              (cphi * stheta * spsi - sphi * cpsi) * input[2];
  output[2] = -stheta * input[0] + sphi * ctheta * input[1] + cphi * ctheta * input[2];

  return output;
}
/**
 * @brief Rotate input vector back around Roll, Pitch, Yaw
 *        This is the invers transformation to rotate_ypr
 *        (e.g. vehicle frame to local cartesian)
 *
 * @param[in] input         - Input vector to be rotated
 *
 * @param[in] euler_ypr     - Euler angles
 */
inline tam::types::common::Vector3D<double> rotate_inv_rpy(
  const tam::types::common::Vector3D<double> & input,
  const tam::types::common::EulerYPR & euler_ypr)
{
  tam::types::common::Vector3D<double> output;

  double sphi = std::sin(euler_ypr.roll), cphi = std::cos(euler_ypr.roll);
  double stheta = std::sin(euler_ypr.pitch), ctheta = std::cos(euler_ypr.pitch);
  double spsi = std::sin(euler_ypr.yaw), cpsi = std::cos(euler_ypr.yaw);

  output.x = ctheta * cpsi * input.x + ctheta * spsi * input.y - stheta * input.z;
  output.y = (sphi * stheta * cpsi - cphi * spsi) * input.x +
             (sphi * stheta * spsi + cphi * cpsi) * input.y + sphi * ctheta * input.z;
  output.z = (cphi * stheta * cpsi + sphi * spsi) * input.x +
             (cphi * stheta * spsi - sphi * cpsi) * input.y + cphi * ctheta * input.z;

  return output;
}
/**
 * @brief Rotate input vector back around Roll, Pitch, Yaw
 *        This is the invers transformation to rotate_ypr
 *        (e.g. vehicle frame to local cartesian)
 *
 * @param[in] input         - Input vector to be rotated
 *                            idx: 0 - x, 1 - y, 2 - z
 *
 * @param[in] euler_ypr     - Euler angles
 *                            idx: 0 - roll, 1 - pitch, 2 - yaw
 */
inline const Eigen::Vector3d rotate_inv_rpy(
  const Eigen::Ref<const Eigen::Vector3d> & input,
  const Eigen::Ref<const Eigen::Vector3d> & euler_ypr)
{
  Eigen::Vector3d output;

  double sphi = std::sin(euler_ypr[0]), cphi = std::cos(euler_ypr[0]);
  double stheta = std::sin(euler_ypr[1]), ctheta = std::cos(euler_ypr[1]);
  double spsi = std::sin(euler_ypr[2]), cpsi = std::cos(euler_ypr[2]);

  output[0] = ctheta * cpsi * input[0] + ctheta * spsi * input[1] - stheta * input[2];
  output[1] = (sphi * stheta * cpsi - cphi * spsi) * input[0] +
              (sphi * stheta * spsi + cphi * cpsi) * input[1] + sphi * ctheta * input[2];
  output[2] = (cphi * stheta * cpsi + sphi * spsi) * input[0] +
              (cphi * stheta * spsi - sphi * cpsi) * input[1] + cphi * ctheta * input[2];

  return output;
}
inline tam::types::common::EulerYPR rotate_angular_velocities_to_2D(
  const tam::types::common::EulerYPR & input, const tam::types::common::EulerYPR & euler_ypr)
{
  tam::types::common::EulerYPR out(0.0, 0.0, 0.0);
  out.yaw = std::sin(euler_ypr.roll) / std::cos(euler_ypr.pitch) * input.pitch +
            std::cos(euler_ypr.roll) / std::cos(euler_ypr.pitch) * input.yaw;  // dx_psi
  out.pitch =
    std::cos(euler_ypr.roll) * input.pitch - std::sin(euler_ypr.roll) * input.yaw;  // dx_theta
  out.roll = input.roll + std::sin(euler_ypr.roll) * std::tan(euler_ypr.pitch) * input.pitch +
             std::cos(euler_ypr.roll) * std::tan(euler_ypr.pitch) * input.yaw;  // dx_phi
  return out;
}
}  // namespace math
/// @brief Returns accelerations in the footprint view of the vehicle coordinate system.
/// @param input mps2
/// @param pitch in rad
/// @param roll in rad
/// @return
inline Eigen3D vector_to_2d(
  const Eigen3D & input, const Eigen::VectorXd & pitch, const Eigen::VectorXd & roll)
{
  typedef tam::types::common::Vector3D<double> vector_t;
  typedef tam::types::common::EulerYPR euler_t;
  Eigen3D out;
  out.resize(input.x.size());
  for (Eigen::Index i = 0; i < input.x.size(); i++) {
    vector_t out_ = math::rotate_ypr(
      vector_t(input.x[i], input.y[i], input.z[i]), euler_t(0.0, pitch[i], roll[i]));
    out.x[i] = out_.x;
    out.y[i] = out_.y;
    out.z[i] = out_.z;
  }
  return out;
}
/// @brief Returns accelerations in the footprint view of the vehicle coordinate system.
/// @param input mps2
/// @param pitch in rad
/// @param roll in rad
/// @return
inline tam::types::common::Vector3D<double> vector_to_2d(
  const tam::types::common::Vector3D<double> & input, const double pitch, const double roll)
{
  typedef tam::types::common::EulerYPR euler_t;
  return math::rotate_ypr(input, euler_t(0.0, pitch, roll));
}
/// @brief Transforms from footprint view to 3d
/// @param input mps2
/// @param pitch in rad
/// @param roll in rad
/// @return
inline Eigen3D vector_2d_to_3d(
  const Eigen3D & input, const Eigen::VectorXd & pitch, const Eigen::VectorXd & roll)
{
  typedef tam::types::common::Vector3D<double> vector_t;
  typedef tam::types::common::EulerYPR euler_t;
  Eigen3D out;
  out.resize(input.x.size());
  for (Eigen::Index i = 0; i < input.x.size(); i++) {
    vector_t out_ = math::rotate_inv_rpy(
      vector_t(input.x[i], input.y[i], input.z[i]), euler_t(0.0, pitch[i], roll[i]));
    out.x[i] = out_.x;
    out.y[i] = out_.y;
    out.z[i] = out_.z;
  }
  return out;
}
/// @brief Transforms from footprint view to 3d
/// @param input mps2
/// @param pitch in rad
/// @param roll in rad
/// @return
inline tam::types::common::Vector3D<double> vector_2d_to_3d(
  const tam::types::common::Vector3D<double> & input, const double pitch, const double roll)
{
  typedef tam::types::common::EulerYPR euler_t;
  return math::rotate_inv_rpy(input, euler_t(0.0, pitch, roll));
}
/// @brief Returns angular velocities in the footprint view of the vehicle coordinate system.
/// @param angular_velocities radps
/// @param pitch in rad
/// @param roll in rad
/// @return
inline Eigen3D angular_velocities_to_2d(
  const Eigen3D & angular_velocities, const Eigen::VectorXd & pitch, const Eigen::VectorXd & roll)
{
  typedef tam::types::common::EulerYPR euler_t;
  Eigen3D out;
  out.resize(angular_velocities.x.size());
  for (Eigen::Index i = 0; i < angular_velocities.x.size(); i++) {
    euler_t out_ = math::rotate_angular_velocities_to_2D(
      euler_t(angular_velocities.x[i], angular_velocities.y[i], angular_velocities.z[i]),
      euler_t(0.0, pitch[i], roll[i]));
    out.x[i] = out_.yaw;
    out.y[i] = out_.pitch;
    out.z[i] = out_.roll;
  }
  return out;
}
/// @brief Returns angular velocities in the footprint view of the vehicle coordinate system.
/// @param angular_velocities radps
/// @param pitch in rad
/// @param roll in rad
/// @return
inline tam::types::common::EulerYPR angular_velocities_to_2d(
  const tam::types::common::EulerYPR & angular_velocities, const double pitch, const double roll)
{
  typedef tam::types::common::EulerYPR euler_t;
  return math::rotate_angular_velocities_to_2D(angular_velocities, euler_t(0.0, pitch, roll));
}
}  // namespace tam::helpers::euler_rotations
