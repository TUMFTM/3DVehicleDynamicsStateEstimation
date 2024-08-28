// Copyright 2023 TUM FTM
#pragma once
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>
namespace tam::core
{
struct TimeoutDescriptor
{
  std::function<void(bool, std::chrono::milliseconds)> timeout_callback;
  std::chrono::milliseconds timeout;
  rclcpp::Time last_update;
  TimeoutDescriptor(
    std::function<void(bool, std::chrono::milliseconds)> timeout_callback_,
    std::chrono::milliseconds timeout_, rclcpp::Time last_update_)
  : timeout_callback{timeout_callback_}, timeout{timeout_}, last_update{last_update_}
  {
  }
};
/**
 * Class for a watchdog handler.
 *
 * Subscriptions can be created via this class that are automatically monitored for their timeout.
 *
 */
class TopicWatchdog
{
private:
  rclcpp::Node * node;
  std::vector<std::shared_ptr<TimeoutDescriptor>> watched_callbacks;
  // Just store them so they don't go out of scope
  std::vector<rclcpp::SubscriptionBase::SharedPtr> created_ros_subs;

public:
  void check_timeouts();
  explicit TopicWatchdog(rclcpp::Node * node);
  /**
   * @param topic returns a std::function of the TopicWatchdogs update function.
   */
  std::function<void()> get_update_function();
  /**
   * @param topic Name of the subscribed topic
   * @param qos QoS settings of the subscription
   * @param subscription_callback Function that handles incoming messages, will be passed to the
   * underlying ROS 2 subscription
   * @param timeout_callback Function that is called whenever a timeout is detected
   * @param timeout Timeout for the subscription, e.g. 500ms
   * @returns Subscription Handle of the underlying ROS 2 subscription
   */

  template <typename CallbackT>
  auto timeout_callback(
    CallbackT && callback, std::function<void(bool, std::chrono::milliseconds)> timeout_callback,
    std::chrono::milliseconds timeout)
  {
    auto timeout_descriptor = std::make_shared<TimeoutDescriptor>(
      TimeoutDescriptor{timeout_callback, timeout, this->node->get_clock()->now()});
    this->watched_callbacks.emplace_back(timeout_descriptor);

    return
      [callback = std::forward<CallbackT>(callback), timeout_descriptor, this](auto &&... args) {
        timeout_descriptor->last_update = this->node->get_clock()->now();
        return callback(std::forward<decltype(args)>(args)...);
      };
  }
  template <typename T, typename CallbackT>
  typename rclcpp::Subscription<T>::SharedPtr add_subscription(
    std::string topic, const rclcpp::QoS & qos, CallbackT && subscription_callback,
    std::function<void(bool, std::chrono::milliseconds)> timeout_callback,
    std::chrono::milliseconds timeout)
  {
    // Create struct now so it can be referenced in subscription callback
    auto timeout_descriptor = std::make_shared<TimeoutDescriptor>(
      TimeoutDescriptor{timeout_callback, timeout, this->node->get_clock()->now()});

    /**
     * Use this to allow accepting e.g. a SharedPtr of T
     * Without this the callback type is forced to be of type T
     * Subscriptions work with shared pointers to messages
     */
    using rclcpp::AnySubscriptionCallback;
    // Shared pointer is necessary as all bound values have to be const
    // Calling dispatch on the callback is not allowed otherwise
    std::shared_ptr<AnySubscriptionCallback<T>> any_subscription_callback =
      std::make_shared<AnySubscriptionCallback<T>>();
    any_subscription_callback->set(std::forward<CallbackT>(subscription_callback));

    /**
     * Create subscription for the node
     * Bind "this" and the struct to update the relevant last update field automatically on sub
     * Add subscription reference to struct so it doesn't get cleaned up
     */
    typename rclcpp::Subscription<T>::SharedPtr ros_sub = node->create_subscription<T>(
      topic, qos,
      [this, timeout_descriptor, any_subscription_callback](
        std::shared_ptr<T> msg, const rclcpp::MessageInfo & message_info) {
        timeout_descriptor->last_update = this->node->get_clock()->now();
        any_subscription_callback->dispatch(msg, message_info);
      });
    created_ros_subs.push_back(ros_sub);

    // Add watchdog subscription to list
    this->watched_callbacks.emplace_back(timeout_descriptor);

    return ros_sub;
  }
  using SharedPtr = std::shared_ptr<tam::core::TopicWatchdog>;
  using UniquePtr = std::unique_ptr<tam::core::TopicWatchdog>;
};
}  // namespace tam::core
