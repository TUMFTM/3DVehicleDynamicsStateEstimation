#include <rclcpp/rclcpp.hpp>
#include "state_estimation_node_cpp/state_estimation_node_impl.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<stateEstimationNode<tam::core::state::EKF_2D>>(
    std::make_unique<tam::core::state::StateEstimation<tam::core::state::EKF_2D>>("non-holonomic"),
    "non-holonomic", options);
  tam::ros::validate_param_overrides(argc, argv, node.get());
  rclcpp::executors::StaticSingleThreadedExecutor single_thread_executor;
  single_thread_executor.add_node(node);
  single_thread_executor.spin();
  rclcpp::shutdown();
  return 0;
}
