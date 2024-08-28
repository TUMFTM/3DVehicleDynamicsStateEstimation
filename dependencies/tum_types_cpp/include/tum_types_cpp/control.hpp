// Copyright 2023 Simon Sagmeister
#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "tum_types_cpp/common.hpp"
namespace tam::types::control
{
struct Odometry
{
  tam::types::common::Vector3D<double> position_m;
  tam::types::common::Vector3D<double>
    orientation_rad;  // Euler Angles - therefore the assignement is (x:roll, y:pitch, z:yaw)
  /*6x6 covariance matrix
  The orientation parameters use a fixed-axis representation.
  In order, the parameters are:
  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
  std::array<double, 36> pose_covariance;  //  Row Major Matrix Representation

  tam::types::common::Vector3D<double>
    velocity_mps;  // In the "local" coordinate system speficied by position and orientation
  tam::types::common::Vector3D<double>
    angular_velocity_radps;  // In the "local" coordinate system speficied by
                             // position and orientation
  /*6x6 covariance matrix
  The orientation parameters use a fixed-axis representation.
  In order, the parameters are:
  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
  std::array<double, 36> velocity_covariance;  //  Row Major Matrix Representation
};
struct AccelerationwithCovariances
{
  tam::types::common::Vector3D<double>
    acceleration_mps2;  // In the "local" coordinate system speficied by position and orientation
  tam::types::common::Vector3D<double>
    angular_acceleration_radps2;  // In the "local" coordinate system speficied by
                                  // position and orientation
  /*6x6 covariance matrix
  The orientation parameters use a fixed-axis representation.
  In order, the parameters are:
  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
  std::array<double, 36> acceleration_covariance;  //  Row Major Matrix Representation
};
};  // namespace tam::types::control
