// Copyright 2023 Simon Sagmeister
#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
namespace tam::helpers::numerical
{
template <typename _ForwardIterator>
inline int find_bottom_idx(_ForwardIterator __first, _ForwardIterator __last, const double x)
{
  auto iter_geq = std::lower_bound(__first, __last, x);
  if (iter_geq == __first) {
    return 0;
  }
  if (iter_geq == __last) {
    return iter_geq - __first - 2;
  }
  return iter_geq - __first - 1;
}
// xp muss monoton steigend sein
template <typename Ta, typename Tb>
inline double interp(const double x, const Ta & xp, const Tb & fp)
{
  if ((xp.end() - xp.begin()) != (fp.end() - fp.begin())) {
    throw std::invalid_argument("<tam::helper::geometry::interp> xp.size() != fp.size()");
  }
  int i = find_bottom_idx(xp.begin(), xp.end(), x);
  // f = fp(i) + ((x - xp(i)) / (xp(i + 1) - xp(i))) * (fp(i + 1) - fp(i));
  auto fp_idx = fp.begin();
  auto xp_idx = xp.begin();
  return *(fp_idx + i) + ((x - *(xp_idx + i)) / (*(xp_idx + i + 1) - *(xp_idx + i))) *
                           (*(fp_idx + i + 1) - *(fp_idx + i));
}
template <typename Ta, typename Tb>
inline std::vector<double> interp(const std::vector<double> & x, const Ta & xp, const Tb & fp)
{
  std::vector<double> f;
  f.reserve(x.size());
  for (const auto & x_ : x) {
    f.push_back(interp(x_, xp, fp));
  }
  return f;
}
template <typename Ta, typename Tb>
inline Eigen::MatrixXd interp(
  const Eigen::Ref<const Eigen::MatrixXd> x, const Ta & xp, const Tb & fp)
{
  Eigen::MatrixXd f;
  f.resize(x.rows(), x.cols());
  auto f_it = f.reshaped();
  auto x_it = x.reshaped();
  for (int i = 0; i < f.size(); ++i) {
    f_it(i) = interp(x_it(i), xp, fp);
  }
  return f_it.reshaped(x.rows(), x.cols());
}
}  // namespace tam::helpers::numerical
