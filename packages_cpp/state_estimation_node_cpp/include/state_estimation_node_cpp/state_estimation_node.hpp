// Copyright 2023 TUMWFTM
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cmath>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// type definitions
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/data_per_wheel.hpp"
#include "tum_types_cpp/control.hpp"
#include "tum_helpers_cpp/containers.hpp"

// 3d to 2d helper functions
#include "tum_helpers_cpp/rotations.hpp"

// type conversions
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
#include "tum_type_conversions_ros_cpp/orientation.hpp"

// messages
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "msgs/msg/tum_float64_per_wheel.hpp"
#include "msgs/msg/tum_float64_per_wheel_stamped.hpp"
#include "msgs/msg/steering_report.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

// state estimation
#include "state_estimation_base/state_estimation_base.hpp"
#include "state_estimation_cpp/state_estimation.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

// param manager
#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager_composer.hpp"
#include "ros_param_helpers_cpp/helper_functions.hpp"

// debug helper
#include "ros_debug_helpers_cpp/debug_publisher.hpp"

// topic watchdog
#include "ros2_watchdog_cpp/topic_watchdog.hpp"

// undef simulink macros
#undef ERROR
#undef WARN
#undef OK

template <class TConfig>
class stateEstimationNode : public rclcpp::Node
{
public:
  explicit stateEstimationNode(
    std::unique_ptr<tam::core::state::StateEstimationBase> && state_estimation,
    const std::string vehicle_model, const rclcpp::NodeOptions & options);

private:
  /**
   * @brief struct defining the number and name of the dubug channels
   */
  struct debug_channels
  {
    enum Channel
      { odometry_1, odometry_2, odometry_3, state_machine, kalman_filter, NUM_DEBUG_CHANNELS };
  };

  // Timer callbacks
  /**
   * @brief Step the state estimation once and publish the outputs
   */
  void model_update_callback(void);

  // publisher function
  /**
   * @brief publish odometry output of the state estimation as nav_msgs::msg::Odometry
   */
  void publish_odometry(rclcpp::Time time_pub);

  /**
   * @brief publish linear acceleration output of the state estimation as
   *        geometry_msgs::msg::AccelWithCovarianceStamped
   */
  void publish_acceleration(rclcpp::Time time_pub);

  /**
   * @brief publish dynamic transforms:
   *            local_cartesian -> vehicle_velocity_cg
   *            vehicle_cg      -> vehicle_velocity_cg
   */
  void publish_dynamic_transforms(void);

  /**
   * @brief publish the transformed odometry input
   * 
   * @param[in] channel             - debug_channels::Channel:
   *                                  debug channel to publish on
   * @param[in] odometry            - tam::types::control::Odometry:
   *                                  odmetry that should be published
   */
  void publish_debug_odometry_input(
    typename debug_channels::Channel channel, const tam::types::control::Odometry & odometry);

  /**
   * @brief update the road angles based on the prior knowledge on the road and the current vehicle position
   * // e.g. with an additional track handler - not implemented in this version
   *
   * @param[in] odometry            - tam::types::control::Odometry:
   *                                  odmetry that should be used to get the road angles
   */
  void update_road_angles(const tam::types::control::Odometry & odometry);

  // Subscription callbacks sensors
  void odometry_1_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imu_1_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void status_1_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg);

  void odometry_2_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imu_2_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void status_2_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg);

  void odometry_3_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imu_3_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void status_3_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg);

  void wheelspeed_report_callback(const msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg);
  void wheelspeed_status_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg);

  void side_slip_estimation_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void side_slip_estimation_status_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg);

  void steering_report_callback(
    const msgs::msg::SteeringReport::SharedPtr msg);

  // Timeout callback for the sensor subscription
  void odometry_1_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void status_1_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void imu_1_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);

  void odometry_2_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void status_2_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void imu_2_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);

  void odometry_3_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void status_3_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void imu_3_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);

  void wheelspeed_report_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void wheelspeed_status_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);

  void steering_report_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);

  void side_slip_estimation_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void side_slip_estimation_status_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);

  /**
   * @brief state estimation class
   */
  std::unique_ptr<tam::core::state::StateEstimationBase> state_estimation_;

  /**
   * @brief parameter manager
   */
  std::shared_ptr<tam::interfaces::ParamManagerBase> param_manager_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  /**
   * @brief model update callback
   */
  rclcpp::TimerBase::SharedPtr model_update_timer_;

  /**
   * @brief topic watchdog
   */
  tam::core::TopicWatchdog::UniquePtr topic_watchdog_;

  /**
   * @brief callback function queue
   */
  tam::core::function_queue<void()> callback_queue_;

  /**
   * @brief tf2 transform broadcaster
   */
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /**
   * @brief tf2 transform listener
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
   * @brief roll and pitch of the road surface wrt. to the current vehicle position (heading)
   */
  tam::types::common::Vector3D<double> road_angles_;

  /**
   * @brief initialize the initial state in the state estimation
   */
  bool initialize_ = true;

  // state estimation output
  /**
   * @brief state estimation odometry output
   */
  tam::types::control::Odometry odometry_output_;

  /**
   * @brief state estimation acceleration output
   */
  tam::types::control::AccelerationwithCovariances acceleration_output_;

  // Publishers
  /**
   * @brief nav_msgs::msg::Odometry publisher
   */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;

  /**
   * @brief geometry_msgs::msg::AccelWithCovarianceStamped publisher
   */
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr pub_acceleration_;

  /**
   * @brief array containing all debug channels
   */
  std::array<tam::ros::DebugPublisher, debug_channels::NUM_DEBUG_CHANNELS>
    debug_channel_publishers_ = {
      tam::ros::DebugPublisher(this, "odometry_1"),
      tam::ros::DebugPublisher(this, "odometry_2"),
      tam::ros::DebugPublisher(this, "odometry_3"),
      tam::ros::DebugPublisher(this, "state_machine"),
      tam::ros::DebugPublisher(this, "kalman_filter"),
    };

  /**
   * @brief additional debug containers to publish the transformed odometry input
   */
  std::array<tam::types::common::TUMDebugContainer::SharedPtr, 3>
    additional_debug_containers_{
      std::make_shared<tam::types::common::TUMDebugContainer>(),
      std::make_shared<tam::types::common::TUMDebugContainer>(),
      std::make_shared<tam::types::common::TUMDebugContainer>()
    };
};
#include "state_estimation_node_cpp/state_estimation_node_impl.hpp"
