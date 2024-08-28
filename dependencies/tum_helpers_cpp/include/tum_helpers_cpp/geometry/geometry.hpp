// Copyright 2023 Simon Hoffmann
#pragma once
#include <math.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

#include "tum_types_cpp/common.hpp"
namespace tam::helpers::geometry
{
inline double normalize_angle(double angle_rad)
{
  /*Modifies an angle to garantue that is lies in the interval [-pi,pi[

  Taken from: https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code*/
  angle_rad = fmod(angle_rad + M_PI, 2 * M_PI);
  if (angle_rad < 0) angle_rad += 2 * M_PI;
  return angle_rad - M_PI;
}
inline Eigen::MatrixXd normalize_angle(const Eigen::Ref<const Eigen::MatrixXd> angle_rad)
{
  Eigen::MatrixXd angle_normalized(angle_rad.rows(), angle_rad.cols());
  auto angle_normalized_it = angle_normalized.reshaped();
  auto angle_rad_it = angle_rad.reshaped();
  for (int i = 0; i < angle_rad.size(); ++i) {
    angle_normalized_it(i) = normalize_angle(angle_rad_it(i));
  }
  return angle_normalized_it.reshaped(angle_rad.rows(), angle_rad.cols());
}
inline std::vector<double> create_s_coordinate_from_points(
  const std::vector<double> & x, const std::vector<double> & y, const std::vector<double> & z)
{
  double length{0};
  std::vector<double> out;
  out.reserve(x.size());
  out.push_back(0.0);
  for (int i = 0; i < static_cast<int>(x.size() - 1); ++i) {
    Eigen::Vector3d pt_1(x.at(i), y.at(i), z.at(i));
    Eigen::Vector3d pt_2(x.at(i + 1), y.at(i + 1), z.at(i + 1));
    length += (pt_2 - pt_1).norm();
    out.push_back(length);
  }
  return out;
}
inline std::vector<double> calc_segment_length(const std::vector<double> & s)
{
  std::vector<double> out;
  out.reserve(s.size() - 1);  // one less segment compared to points
  for (auto it = s.begin() + 1; it < s.end(); ++it) {
    out.push_back(std::abs(*it - *std::prev(it, 1)));
  }
  return out;
}
inline double euclidean_distance(const double x1, const double y1, const double x2, const double y2)
{
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}
inline double euclidean_norm(const tam::types::common::Vector3D<double> & vector)
{
  return sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
}
inline double calc_heading(
  const Eigen::Vector2d & dir, const Eigen::Vector2d & zero_axis = Eigen::Vector2d(1, 0))
{
  return atan2(
    zero_axis[0] * dir[1] - zero_axis[1] * dir[0], zero_axis[0] * dir[0] + zero_axis[1] * dir[1]);
}
inline double to_deg(const double angle) { return angle * (180.0 / M_PIl); }
inline double to_rad(const double angle) { return angle * (M_PIl / 180.0); }
inline bool in_interval(const double value, const double interval_begin, const double interval_end)
{
  if ((interval_begin > interval_end) && (interval_begin <= value || value <= interval_end)) {
    return true;
  } else if (interval_begin <= value && value <= interval_end) {
    return true;
  } else {
    return false;
  }
}
inline bool in_interval(const double value, const std::vector<double> & interval)
{
  return in_interval(value, interval.at(0), interval.at(1));
}
inline double get_curvature_from_points(
  const tam::types::common::Vector2D<double> & pt1,
  const tam::types::common::Vector2D<double> & pt2,
  const tam::types::common::Vector2D<double> & pt3)
{
  // https://hratliff.com/files/curvature_calculations_and_circle_fitting.pdf
  double den = 2 * ((pt2.x - pt1.x) * (pt3.y - pt2.y) - (pt2.y - pt1.y) * (pt3.x - pt2.x));
  double num = std::pow(
    (std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2)) *
      (std::pow(pt3.x - pt2.x, 2) + std::pow(pt3.y - pt2.y, 2)) *
      (std::pow(pt1.x - pt3.x, 2) + std::pow(pt1.y - pt3.y, 2)),
    0.5);

  if (std::abs(num) < 1e-14) {
    return 1e14;  // if numerator goes to zero -> high curvature
  }
  return den / num;  // R=num/den
}
inline double iac_mod(const double a, const double n) { return (a - floor(a / n) * n); }
inline double get_delta_angle(const double yaw_1, const double yaw_2)
{
  double a = normalize_angle(yaw_1) - normalize_angle(yaw_2);
  return iac_mod((a + M_PI_2), M_PI) - M_PI_2;
}
inline double get_curvature_from_heading(
  const double heading_curr, const double heading_nxt, const double s_curr, const double s_nxt)
{
  return get_delta_angle(heading_nxt, heading_curr) / (s_nxt - s_curr);
}
inline double get_curvature_from_heading(
  const double heading_prev, const double heading_curr, const double heading_nxt,
  const double s_prev, const double s_curr, const double s_nxt)
{
  double kappa_nxt = get_delta_angle(heading_nxt, heading_curr) / (s_nxt - s_curr);
  double kappa_prev = get_delta_angle(heading_curr, heading_prev) / (s_curr - s_prev);
  return 0.5 * (kappa_nxt + kappa_prev);
}
inline Eigen::VectorXd get_curvature_from_points(
  const Eigen::Ref<const Eigen::VectorXd> x, const Eigen::Ref<const Eigen::VectorXd> y)
{
  Eigen::VectorXd kappa;
  kappa.resize(x.size());
  for (Eigen::Index i = 1; i < x.size() - 1; i++) {
    tam::types::common::Vector2D<double> pt1{x[i - 1], y[i - 1]}, pt2{x[i], y[i]},
      pt3{x[i + 1], y[i + 1]};
    kappa[i] = tam::helpers::geometry::get_curvature_from_points(pt1, pt2, pt3);
  }
  kappa[0] = kappa[1];
  kappa[x.size() - 1] = kappa[x.size() - 2];
  return kappa;
}
inline Eigen::VectorXd get_curvature_from_heading(
  const Eigen::Ref<const Eigen::VectorXd> yaw, const Eigen::Ref<const Eigen::VectorXd> s)
{
  Eigen::VectorXd kappa;
  kappa.resize(yaw.size());
  for (Eigen::Index i = 1; i < yaw.size() - 1; i++) {
    kappa[i] = tam::helpers::geometry::get_curvature_from_heading(
      yaw[i - 1], yaw[i], yaw[i + 1], s[i - 1], s[i], s[i + 1]);
  }
  kappa[0] = tam::helpers::geometry::get_curvature_from_heading(yaw[0], yaw[1], s[0], s[1]);
  kappa[yaw.size() - 1] = tam::helpers::geometry::get_curvature_from_heading(
    yaw[yaw.size() - 2], yaw[yaw.size() - 1], s[yaw.size() - 2], s[yaw.size() - 1]);
  return kappa;
}
}  // namespace tam::helpers::geometry
