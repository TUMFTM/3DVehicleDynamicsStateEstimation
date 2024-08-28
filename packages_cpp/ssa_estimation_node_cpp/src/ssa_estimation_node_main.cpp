#include <rclcpp/rclcpp.hpp>
#include "ssa_estimation_node_cpp/ssa_estimation_node_impl.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<SSAEstimationNode<tam::core::ssa::UKF_STM>>(
    std::make_unique<tam::core::ssa::SSAEstimation<tam::core::ssa::UKF_STM>>(), options);
  tam::ros::validate_param_overrides(argc, argv, node.get());
  rclcpp::executors::StaticSingleThreadedExecutor single_thread_executor;
  single_thread_executor.add_node(node);
  single_thread_executor.spin();
  rclcpp::shutdown();
  return 0;
}
