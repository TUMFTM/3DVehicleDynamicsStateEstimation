#include "ros2_watchdog_cpp/topic_watchdog.hpp"
namespace tam::core
{
/**
 * Create a Watchdog that periodically checks timeout of subscriptions
 *
 * @param node ROS 2 node that his watchdog is running in
 */
TopicWatchdog::TopicWatchdog(rclcpp::Node * node) { this->node = node; }
std::function<void()> TopicWatchdog::get_update_function()
{
  return std::bind(&TopicWatchdog::check_timeouts, this);
}
void TopicWatchdog::check_timeouts()
{
  auto now = this->node->get_clock()->now();
  for (auto & watchdog_sub : this->watched_callbacks) {
    std::chrono::milliseconds timeout_now =
      (now - watchdog_sub->last_update).to_chrono<std::chrono::milliseconds>();
    watchdog_sub->timeout_callback(timeout_now > watchdog_sub->timeout, timeout_now);
  }
}
}  // namespace tam::core
