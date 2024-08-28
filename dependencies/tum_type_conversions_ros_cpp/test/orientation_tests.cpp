// Copyright 2023 Simon Hoffmann
#include <gtest/gtest.h>

#include <vector>

#include "tum_helpers_cpp/geometry/geometry.hpp"
#include "tum_type_conversions_ros_cpp/orientation.hpp"
TEST(orientation_conversion_tests, euler_quaternion_conversion)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 1.0;
  q.y = 0.4;
  q.z = 0.6;
  q.w = 0.8;

  auto euler_calc = tam::types::conversion::quaternion_msg_to_euler_type(q);
  ASSERT_LE(std::abs(euler_calc.yaw - 1.0040671), 0.00001);
  ASSERT_LE(std::abs(euler_calc.pitch - (-0.2622552)), 0.00001);
  ASSERT_LE(std::abs(euler_calc.roll - 1.6475682), 0.00001);
}
TEST(orientation_conversion_tests, euler_quaternion_conversion_scale)
{
  std::vector<double> test_angles{-M_PI, -0.75 * M_PI, -0.25 * M_PI, 0, 0.25 * M_PI, 0.75 * M_PI};
  test_angles.push_back(M_PI);

  std::vector<double> test_angles2{0.3 * M_PI, -0.25 * M_PI, 0, 0.25 * M_PI, 0.3 * M_PI};

  for (auto yaw : test_angles) {
    for (auto pitch : test_angles2) {
      for (auto roll : test_angles2) {
        auto q = tam::types::conversion::euler_type_to_quaternion_msg(
          tam::types::common::EulerYPR(yaw, pitch, roll));
        auto euler_calc = tam::types::conversion::quaternion_msg_to_euler_type(q);
        ASSERT_LE(std::abs(euler_calc.yaw - yaw), 1e-11);
        ASSERT_LE(std::abs(euler_calc.pitch - pitch), 1e-11);
        ASSERT_LE(std::abs(euler_calc.roll - roll), 1e-11);
      }
    }
  }
}
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
