// Copyright 2023 TUMWFTM
#pragma once
#include "state_estimation_node_cpp/state_estimation_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

template <class TConfig> stateEstimationNode<TConfig>::stateEstimationNode(
  std::unique_ptr<tam::core::state::StateEstimationBase> && state_estimation,
  const std::string vehicle_model, const rclcpp::NodeOptions & options)
: Node("StateEstimation", "/core/state", options), state_estimation_(std::move(state_estimation))
{
  // Initialize the transform broadcaster
  // has to be done before initializing the param manager
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // initialize TransformListener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize the ParamManagerBase
  param_manager_ = state_estimation_->get_param_handler();

  callback_handle_ = tam::ros::connect_param_manager_to_ros_cb(this, param_manager_.get());
  tam::ros::declare_ros_params_from_param_manager(this, param_manager_.get());

  // Check Parameter
  for (std::string name : param_manager_->list_parameters()) {
    rclcpp::Parameter param = this->get_parameter(name);
    RCLCPP_INFO(
      this->get_logger(), "param name: %s, value: %s", param.get_name().c_str(),
      param.value_to_string().c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Parameter Manager of State Estimation initialized");

  // initialize the topic watchdog
  std::function<void()> timer_callback_fct
    = std::bind(&stateEstimationNode::model_update_callback, this);
  topic_watchdog_ = std::make_unique<tam::core::TopicWatchdog>(this);
  callback_queue_.push_back(topic_watchdog_->get_update_function());
  callback_queue_.push_back(timer_callback_fct);

  // model step callback
  model_update_timer_ =
    this->create_wall_timer(10ms, callback_queue_.get_function_queue());

  // subscriber
  topic_watchdog_->add_subscription<nav_msgs::msg::Odometry>(
    "/vehicle/sensor/odometry1", 10, std::bind(&stateEstimationNode::odometry_1_callback, this, _1),
    std::bind(&stateEstimationNode::odometry_1_timeout_callback, this, _1, _2), 500ms);

  topic_watchdog_->add_subscription<sensor_msgs::msg::Imu>(
    "/vehicle/sensor/imu1", 10, std::bind(&stateEstimationNode::imu_1_callback, this, _1),
    std::bind(&stateEstimationNode::imu_1_timeout_callback, this, _1, _2), 500ms);

  topic_watchdog_->add_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    "/vehicle/sensor/status1", 10, std::bind(&stateEstimationNode::status_1_callback, this, _1),
    std::bind(&stateEstimationNode::status_1_timeout_callback, this, _1, _2), 500ms);

  topic_watchdog_->add_subscription<nav_msgs::msg::Odometry>(
    "/vehicle/sensor/odometry2", 10, std::bind(&stateEstimationNode::odometry_2_callback, this, _1),
    std::bind(&stateEstimationNode::odometry_2_timeout_callback, this, _1, _2), 500ms);

  topic_watchdog_->add_subscription<sensor_msgs::msg::Imu>(
    "/vehicle/sensor/imu2", 10, std::bind(&stateEstimationNode::imu_2_callback, this, _1),
    std::bind(&stateEstimationNode::imu_2_timeout_callback, this, _1, _2), 500ms);

  topic_watchdog_->add_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    "/vehicle/sensor/status2", 10, std::bind(&stateEstimationNode::status_2_callback, this, _1),
    std::bind(&stateEstimationNode::status_2_timeout_callback, this, _1, _2), 500ms);

  topic_watchdog_->add_subscription<nav_msgs::msg::Odometry>(
    "/vehicle/sensor/odometry3", 10, std::bind(&stateEstimationNode::odometry_3_callback, this, _1),
    std::bind(&stateEstimationNode::odometry_3_timeout_callback, this, _1, _2), 500ms);

  topic_watchdog_->add_subscription<sensor_msgs::msg::Imu>(
    "/vehicle/sensor/imu3", 10, std::bind(&stateEstimationNode::imu_3_callback, this, _1),
    std::bind(&stateEstimationNode::imu_3_timeout_callback, this, _1, _2), 500ms);

  topic_watchdog_->add_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    "/vehicle/sensor/status3", 10, std::bind(&stateEstimationNode::status_3_callback, this, _1),
    std::bind(&stateEstimationNode::status_3_timeout_callback, this, _1, _2), 500ms);

  if (vehicle_model != "kinematic") {
    topic_watchdog_->add_subscription<msgs::msg::TUMFloat64PerWheelStamped>(
      "/vehicle/sensor/wheelspeed_radps", 10,
      std::bind(&stateEstimationNode::wheelspeed_report_callback, this, _1),
      std::bind(&stateEstimationNode::wheelspeed_report_timeout_callback, this, _1, _2), 500ms);

    topic_watchdog_->add_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
      "/vehicle/sensor/wheelspeed_status", 10,
      std::bind(&stateEstimationNode::wheelspeed_status_callback, this, _1),
      std::bind(&stateEstimationNode::wheelspeed_status_timeout_callback, this, _1, _2), 500ms);
  }

  if (vehicle_model == "single-track-model") {
    topic_watchdog_->add_subscription<msgs::msg::SteeringReport>(
      "/vehicle/sensor/steering_report", 10,
      std::bind(&stateEstimationNode::steering_report_callback, this, _1),
      std::bind(&stateEstimationNode::steering_report_timeout_callback, this, _1, _2), 500ms);
  }

  topic_watchdog_->add_subscription<nav_msgs::msg::Odometry>(
    "/core/ssa/odometry", 10, std::bind(&stateEstimationNode::side_slip_estimation_callback, this, _1),
    std::bind(&stateEstimationNode::side_slip_estimation_timeout_callback, this, _1, _2), 500ms);
  
  topic_watchdog_->add_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    "/core/ssa/status", 10, std::bind(&stateEstimationNode::side_slip_estimation_status_callback, this, _1),
    std::bind(&stateEstimationNode::side_slip_estimation_status_timeout_callback, this, _1, _2), 500ms);

  // publisher for the state estimation output
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/core/state/odometry", 10);

  pub_acceleration_ = this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "/core/state/acceleration", 10);
}

/**
 * @brief Step the state estimation once and publish the outputs
 */
template <class TConfig> void stateEstimationNode<TConfig>::model_update_callback()
{
  // initialize the initial position and heading of the state estimation
  // set set_initial_state will return true if the state was set successfull
  // we dont publish any message before the state estimation was initialized successfully
  if (initialize_) {
    if (state_estimation_->set_initial_state()) {
      initialize_ = false;
      RCLCPP_INFO(this->get_logger(), "Initial state of the state estimation has been set");
    }
  } else {
    // the state estimation was initialized successfully
    // step the state estimation
    state_estimation_->step();

    // get the state estimation output
    odometry_output_ = state_estimation_->get_odometry();
    acceleration_output_ = state_estimation_->get_acceleration();

    rclcpp::Time time_pub = get_clock()->now();

    // publish state estimation output
    publish_odometry(time_pub);
    publish_acceleration(time_pub);

    // publish dynamic transforms
    publish_dynamic_transforms();

    // publish debug messages of the model
    debug_channel_publishers_[debug_channels::state_machine].publish_debug_outputs(
      state_estimation_->get_state_machine_debug_output().get());

    debug_channel_publishers_[debug_channels::kalman_filter].publish_debug_outputs(
      state_estimation_->get_kalman_filter_debug_output().get());

    // update the road angles based on the state estimation output
    update_road_angles(odometry_output_);
  }
}

/**
 * @brief publish odometry output of the state estimation as nav_msgs::msg::Odometry
 * 
 *        This function template is used publish the state estimation output
 *        in 3D (vehicle coordinate frame for the velocity)
 */
template <> void stateEstimationNode<tam::core::state::EKF_3D>::publish_odometry(
  rclcpp::Time time_pub)
{
  // convert odometry type to a nav_msgs::msg::Odometry
  nav_msgs::msg::Odometry odometry_msg =
    tam::type_conversions::odometry_msg_from_type(odometry_output_);

  // construct message header
  odometry_msg.header.stamp = time_pub;
  odometry_msg.header.frame_id = "local_cartesian";
  odometry_msg.child_frame_id = "vehicle_cg";

  pub_odometry_->publish(odometry_msg);
}

/**
 * @brief publish odometry output of the state estimation as nav_msgs::msg::Odometry
 * 
 *        This function template is used to directly publish the state estimation output in
 *        the 3D
 */
template <class TConfig> void stateEstimationNode<TConfig>::publish_odometry(
  rclcpp::Time time_pub)
{
  // convert odometry type to a nav_msgs::msg::Odometry
  nav_msgs::msg::Odometry odometry_msg =
    tam::type_conversions::odometry_msg_from_type(odometry_output_);

  // construct message header
  odometry_msg.header.stamp = time_pub;
  odometry_msg.header.frame_id = "local_cartesian";
  odometry_msg.child_frame_id = "vehicle_velocity_cg";

  pub_odometry_->publish(odometry_msg);
}

/**
 * @brief publish linear acceleration output of the state estimation as
 *        geometry_msgs::msg::AccelWithCovarianceStamped
 * 
 *        This function template is used to directly publish the state estimation output in
 *        the 3D
 */
template <class TConfig> void stateEstimationNode<TConfig>::publish_acceleration(
  rclcpp::Time time_pub)
{
  // convert odometry type to a geometry_msgs::msg::AccelWithCovarianceStamped
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration_msg =
    tam::type_conversions::accel_with_covariance_stamped_msg_from_type(acceleration_output_);

  // construct message header
  acceleration_msg.header.stamp = time_pub;
  acceleration_msg.header.frame_id = "vehicle_cg";

  pub_acceleration_->publish(acceleration_msg);
}

/**
 * @brief publish dynamic transforms:
 *            local_cartesian -> vehicle_velocity_cg
 *            vehicle_cg      -> vehicle_velocity_cg
 */
template <class TConfig> void stateEstimationNode<TConfig>::publish_dynamic_transforms(void)
{
  // get timestamp and publish transform from
  // local_cartesian to vehicle_velocity_cg
  geometry_msgs::msg::TransformStamped transform_local_cartesian;
  transform_local_cartesian.header.stamp = this->get_clock()->now();
  transform_local_cartesian.header.frame_id = "local_cartesian";
  transform_local_cartesian.child_frame_id = "base_link";

  // translation between current cog and 0, 0, 0
  transform_local_cartesian.transform.translation.x = odometry_output_.position_m.x;
  transform_local_cartesian.transform.translation.y = odometry_output_.position_m.y;
  transform_local_cartesian.transform.translation.z = odometry_output_.position_m.z;

  // rotate yaw angle around z
  transform_local_cartesian.transform.rotation =
    tam::types::conversion::euler_type_to_quaternion_msg(
    tam::types::common::EulerYPR(
      odometry_output_.orientation_rad.z - state_estimation_->get_sideslip_angle(),
      odometry_output_.orientation_rad.y,
      odometry_output_.orientation_rad.x));

  // publish the transformation to the vehicle CoG
  tf_broadcaster_->sendTransform(transform_local_cartesian);

  // get timestamp and publish transform from
  // vehicle_cg to vehicle_velocity_cg
  geometry_msgs::msg::TransformStamped transform_vehicle_cg;
  transform_vehicle_cg.header.stamp = this->get_clock()->now();
  transform_vehicle_cg.header.frame_id = "vehicle_cg";
  transform_vehicle_cg.child_frame_id = "vehicle_velocity_cg";

  // there is no translation
  transform_vehicle_cg.transform.translation.x = 0.0;
  transform_vehicle_cg.transform.translation.y = 0.0;
  transform_vehicle_cg.transform.translation.z = 0.0;

  // rotate sideslip angle around z
  transform_vehicle_cg.transform.rotation =
    tam::types::conversion::euler_type_to_quaternion_msg(
    tam::types::common::EulerYPR(state_estimation_->get_sideslip_angle(), 0.0, 0.0));

  // publish the transformation for sideslip angle
  tf_broadcaster_->sendTransform(transform_vehicle_cg);
}

/**
 * @brief publish the transformed odometry input
 * 
 * @param[in] channel             - debug_channels::Channel:
 *                                  debug channel to publish on
 * @param[in] odometry            - tam::types::control::Odometry:
 *                                  odmetry that should be published
 */
template <class TConfig> void stateEstimationNode<TConfig>::publish_debug_odometry_input(
  typename debug_channels::Channel channel, const tam::types::control::Odometry & odometry)
{
  additional_debug_containers_[channel]->log("position_m/x", odometry.position_m.x);
  additional_debug_containers_[channel]->log("position_m/y", odometry.position_m.y);
  additional_debug_containers_[channel]->log("position_m/z", odometry.position_m.z);

  additional_debug_containers_[channel]->log("orientation_rad/x", odometry.orientation_rad.x);
  additional_debug_containers_[channel]->log("orientation_rad/y", odometry.orientation_rad.y);
  additional_debug_containers_[channel]->log("orientation_rad/z", odometry.orientation_rad.z);

  debug_channel_publishers_[channel].publish_debug_outputs(
    additional_debug_containers_[channel].get());
}

/**
 * @brief update the road angles based on the prior knowledge on the road and the current vehicle position
 * // e.g. with an additional track handler - not implemented in this version
 *
 * @param[in] odometry            - tam::types::control::Odometry:
 *                                  odmetry that should be used to get the road angles
 */
template <class TConfig> void stateEstimationNode<TConfig>::update_road_angles(
  const tam::types::control::Odometry & odometry)
{
  state_estimation_->set_input_road_angles(
    tam::types::common::Vector3D<double>{0.0, 0.0 , 0.0});
  state_estimation_->set_input_road_angle_status(tam::types::ErrorLvl::WARN);
}

template <class TConfig> void stateEstimationNode<TConfig>::odometry_1_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // input position of the first sensor position
  tam::types::control::Odometry input =
    tam::type::conversions::cpp::Odometry_type_from_msg(msg);

  try
  {
    // Try to look up the transform
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform(
      "gps_antenna_front", "vehicle_cg", tf2::TimePointZero);

    // build the rotation matrix
    tf2::Matrix3x3 rotation_matrix;
    if (initialize_) {
      rotation_matrix.setEulerYPR(input.orientation_rad.z,
                                  input.orientation_rad.y,
                                  input.orientation_rad.x);
    } else {
      rotation_matrix.setEulerYPR(odometry_output_.orientation_rad.z,
                                  odometry_output_.orientation_rad.y,
                                  odometry_output_.orientation_rad.x);
    }

    // rotate the antenna offset
    tf2::Vector3 transformation = rotation_matrix * tf2::Vector3(transform.transform.translation.x,
                                                                 transform.transform.translation.y,
                                                                 transform.transform.translation.z);

    // transform from antenna to vehicle CoG
    input.position_m = input.position_m + tam::types::common::Vector3D<double>
      {transformation.x(), transformation.y(), transformation.z()};
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_INFO(this->get_logger(), "Failed to transform: %s", ex.what());
  }

  // set state estimation localization input
  state_estimation_->set_input_position(input, 0);
  state_estimation_->set_input_orientation(input, 0, false, false, true);

  // publish the transformed inputs for debugging
  publish_debug_odometry_input(debug_channels::odometry_1, input);
}

template <class TConfig> void stateEstimationNode<TConfig>::imu_1_callback(
  const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tam::types::control::AccelerationwithCovariances input_acceleration =
    tam::type_conversions::acceleration_with_covariances_type_from_imu_msg(msg);

  tam::types::control::Odometry input_odometry =
    tam::type_conversions::odometry_type_from_imu_msg(msg);

  // set the imu input valid state OK (0)
  state_estimation_->set_input_imu_status(tam::types::ErrorLvl::OK, 0);

  // set acceleration input
  state_estimation_->set_input_acceleration(input_acceleration, 0);

  // set angular velocity input
  state_estimation_->set_input_angular_velocity(input_odometry, 0);
}

template <class TConfig> void stateEstimationNode<TConfig>::status_1_callback(
  const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
{
  // set state estimation sensor status
  state_estimation_->set_input_position_status(
    tam::type_conversions::error_type_from_diagnostic_level(msg->level), 0);

  state_estimation_->set_input_orientation_status(
    tam::type_conversions::error_type_from_diagnostic_level(msg->level), 0);
}

template <class TConfig> void stateEstimationNode<TConfig>::odometry_2_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // input position of the second sensor position
  tam::types::control::Odometry input =
    tam::type::conversions::cpp::Odometry_type_from_msg(msg);

  try
  {
    // Try to look up the transform
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform(
      "gps_antenna_right", "vehicle_cg", tf2::TimePointZero);

    // build the rotation matrix
    tf2::Matrix3x3 rotation_matrix;
    if (initialize_) {
      rotation_matrix.setEulerYPR(input.orientation_rad.z,
                                  input.orientation_rad.y,
                                  input.orientation_rad.x);
    } else {
      rotation_matrix.setEulerYPR(odometry_output_.orientation_rad.z,
                                  odometry_output_.orientation_rad.y,
                                  odometry_output_.orientation_rad.x);
    }

    // rotate the antenna offset
    tf2::Vector3 transformation = rotation_matrix * tf2::Vector3(transform.transform.translation.x,
                                                                 transform.transform.translation.y,
                                                                 transform.transform.translation.z);

    // transform from antenna to vehicle CoG
    input.position_m = input.position_m + tam::types::common::Vector3D<double>
      {transformation.x(), transformation.y(), transformation.z()};
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_INFO(this->get_logger(), "Failed to transform: %s", ex.what());
  }

  // set state estimation localization input
  state_estimation_->set_input_position(input, 1);
  state_estimation_->set_input_orientation(input, 1, false, false, true);

  // publish the transformed inputs for debugging
  publish_debug_odometry_input(debug_channels::odometry_2, input);
}

template <class TConfig> void stateEstimationNode<TConfig>::imu_2_callback(
  const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tam::types::control::AccelerationwithCovariances input_acceleration =
    tam::type_conversions::acceleration_with_covariances_type_from_imu_msg(msg);

  tam::types::control::Odometry input_odometry =
    tam::type_conversions::odometry_type_from_imu_msg(msg);

  // set the imu input valid state OK (0)
  state_estimation_->set_input_imu_status(tam::types::ErrorLvl::OK, 1);

  // set acceleration input
  state_estimation_->set_input_acceleration(input_acceleration, 1);

  // set angular velocity input
  state_estimation_->set_input_angular_velocity(input_odometry, 1);
}

template <class TConfig> void stateEstimationNode<TConfig>::status_2_callback(
  const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
{
  // set state estimation sensor status
  state_estimation_->set_input_position_status(
    tam::type_conversions::error_type_from_diagnostic_level(msg->level), 1);

  state_estimation_->set_input_orientation_status(
    tam::type_conversions::error_type_from_diagnostic_level(msg->level), 1);
}

template <class TConfig> void stateEstimationNode<TConfig>::odometry_3_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // input position of the first sensor position
  tam::types::control::Odometry input =
    tam::type::conversions::cpp::Odometry_type_from_msg(msg);

  tam::types::common::Vector3D<double> translation = {0.0, 0.0, 0.0};

  try
  {
    // Try to look up the transform
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform(
      "gps_antenna_front", "vehicle_cg", tf2::TimePointZero);

    // build the rotation matrix
    tf2::Matrix3x3 rotation_matrix;
    if (initialize_) {
      rotation_matrix.setEulerYPR(input.orientation_rad.z,
                                  input.orientation_rad.y,
                                  input.orientation_rad.x);
    } else {
      rotation_matrix.setEulerYPR(odometry_output_.orientation_rad.z,
                                  odometry_output_.orientation_rad.y,
                                  odometry_output_.orientation_rad.x);
    }

    // rotate the antenna offset
    tf2::Vector3 transformation = rotation_matrix * tf2::Vector3(transform.transform.translation.x,
                                                                 transform.transform.translation.y,
                                                                 transform.transform.translation.z);

    // transform from antenna to vehicle CoG
    input.position_m = input.position_m + tam::types::common::Vector3D<double>
      {transformation.x(), transformation.y(), transformation.z()};
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_INFO(this->get_logger(), "Failed to transform: %s", ex.what());
  }

  // set state estimation localization input
  state_estimation_->set_input_position(input, 2);
  state_estimation_->set_input_orientation(input, 2, true, true, true);

  // publish the transformed inputs for debugging
  publish_debug_odometry_input(debug_channels::odometry_3, input);
}

template <class TConfig> void stateEstimationNode<TConfig>::imu_3_callback(
  const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tam::types::control::AccelerationwithCovariances input_acceleration =
    tam::type_conversions::acceleration_with_covariances_type_from_imu_msg(msg);

  tam::types::control::Odometry input_odometry =
    tam::type_conversions::odometry_type_from_imu_msg(msg);

  // set the imu input valid state OK (0)
  state_estimation_->set_input_imu_status(tam::types::ErrorLvl::OK, 2);

  // set acceleration input
  state_estimation_->set_input_acceleration(input_acceleration, 2);

  // set angular velocity input
  state_estimation_->set_input_angular_velocity(input_odometry, 2);
}

template <class TConfig> void stateEstimationNode<TConfig>::status_3_callback(
  const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
{
  // set state estimation sensor status
  state_estimation_->set_input_position_status(
    tam::type_conversions::error_type_from_diagnostic_level(msg->level), 2);

  state_estimation_->set_input_orientation_status(
    tam::type_conversions::error_type_from_diagnostic_level(msg->level), 2);
}

template <class TConfig> void stateEstimationNode<TConfig>::wheelspeed_report_callback(
  const msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg)
{
  // input angluar velocity per wheel
  tam::types::common::DataPerWheel<double> input;
  input.front_left = msg->data.front_left;
  input.front_right = msg->data.front_right;
  input.rear_left = msg->data.rear_left;
  input.rear_right = msg->data.rear_right;
  // set the wheelspeed input
  state_estimation_->set_input_wheelspeeds(input);
}

template <class TConfig> void stateEstimationNode<TConfig>::wheelspeed_status_callback(
  const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
{
  // set state estimation sensor status
  state_estimation_->set_input_wheelspeed_status(
    tam::type_conversions::error_type_from_diagnostic_level(msg->level));
}

template <class TConfig> void stateEstimationNode<TConfig>::steering_report_callback(
  const msgs::msg::SteeringReport::SharedPtr msg)
{
  // set the steering angle input and the status
  state_estimation_->set_input_steering_angle(msg->steering_tire_angle);
  state_estimation_->set_input_steering_angle_status(tam::types::ErrorLvl::OK);
}

template <class TConfig> void stateEstimationNode<TConfig>::side_slip_estimation_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tam::types::control::Odometry input =
    tam::type::conversions::cpp::Odometry_type_from_msg(msg);

  // set the steering angle input and the status
  state_estimation_->set_input_linear_velocity(input, 0);
}

template <class TConfig> void stateEstimationNode<TConfig>::side_slip_estimation_status_callback(
  const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
{
  // set state estimation sensor status
  state_estimation_->set_input_linear_velocity_status(
    tam::type_conversions::error_type_from_diagnostic_level(msg->level), 0);
}

template <class TConfig> void stateEstimationNode<TConfig>::odometry_1_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_position_timeout(0);
    state_estimation_->set_orientation_timeout(0);
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::status_1_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_position_timeout(0);
    state_estimation_->set_orientation_timeout(0);
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::imu_1_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_imu_timeout(0);
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::odometry_2_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_position_timeout(1);
    state_estimation_->set_orientation_timeout(1);
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::status_2_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_position_timeout(1);
    state_estimation_->set_orientation_timeout(1);
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::imu_2_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_imu_timeout(1);
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::odometry_3_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_position_timeout(2);
    state_estimation_->set_orientation_timeout(2);
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::status_3_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_position_timeout(2);
    state_estimation_->set_orientation_timeout(2);
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::imu_3_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_imu_timeout(2);
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::wheelspeed_report_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_wheelspeeds_timeout();
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::wheelspeed_status_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_wheelspeeds_timeout();
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::steering_report_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_steering_angle_timeout();
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::side_slip_estimation_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_linear_velocity_timeout(0);
  }
}

template <class TConfig> void stateEstimationNode<TConfig>::side_slip_estimation_status_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    state_estimation_->set_linear_velocity_timeout(0);
  }
}