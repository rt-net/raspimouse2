// Copyright 2018 Geoffrey Biggs, AIST
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "raspimouse2/raspimouse2_component.hpp"

#include <cmath>
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

using namespace std::chrono_literals;

namespace raspimouse2
{

RaspiMouse2::RaspiMouse2()
: rclcpp_lifecycle::LifecycleNode("raspimouse2"),
  odom_(rosidl_generator_cpp::MessageInitialization::ZERO),
  odom_transform_(rosidl_generator_cpp::MessageInitialization::ZERO),
  last_odom_time_(0),
  linear_velocity_(0),
  angular_velocity_(0)
{
  // No construction necessary (node is uninitialised)
}

rcl_lifecycle_transition_key_t RaspiMouse2::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Raspimouse node");

  using namespace std::placeholders;  // for _1, _2, _3...

  linear_velocity_ = 0;
  angular_velocity_ = 0;
  last_odom_time_ = now();

  // Publisher for odometry data
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom");
  odom_.child_frame_id = "base_link";
  odom_.pose.pose.position.x = 0;
  odom_.pose.pose.position.y = 0;
  odom_.pose.pose.orientation.x = 0;
  odom_.pose.pose.orientation.y = 0;
  odom_.pose.pose.orientation.z = 0;
  odom_.pose.pose.orientation.w = 0;
  odom_theta_ = 0;
  // Publisher for odometry transform
  // TODO: When the tf2 API is updated to match rclcpp, re-enable this broadcaster
  //odom_transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    //std::static_pointer_cast<rclcpp::Node>(this->shared_from_this()));
  odom_transform_.header.frame_id = "odom";
  odom_transform_.child_frame_id = "base_link";
  odom_transform_.transform.translation.x = 0;
  odom_transform_.transform.translation.y = 0;
  odom_transform_.transform.rotation.x = 0;
  odom_transform_.transform.rotation.y = 0;
  odom_transform_.transform.rotation.z = 0;
  odom_transform_.transform.rotation.w = 0;
  // Timer for providing the odometry data
  odom_timer_ = create_wall_timer(100ms, std::bind(&RaspiMouse2::publish_odometry, this));
  // Don't actually start publishing odometry data until activated
  odom_timer_->cancel();

  // Subscriber for velocity commands
  velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", std::bind(&RaspiMouse2::velocity_command, this, _1));

  // Motor power control service
  power_service_ = create_service<std_srvs::srv::SetBool>(
      "motor_power", std::bind(&RaspiMouse2::handle_motor_power, this, _1, _2, _3));

  // Watchdog timer to prevent out-of-control robots
  watchdog_timer_ = create_wall_timer(60s, std::bind(&RaspiMouse2::watchdog, this));

  // Publisher for switch states
  switches_pub_ = this->create_publisher<raspimouse2::msg::Switches>("switches");
  // Publisher for light sensors
  light_sensors_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("light_sensors");
  // Timer for publishing sensor information
  sensors_timer_ = create_wall_timer(100ms, std::bind(&RaspiMouse2::publish_sensors, this));
  sensors_timer_->cancel();
  // Subscriber for LED commands
  leds_sub_ = create_subscription<raspimouse2::msg::LEDs>(
      "leds", std::bind(&RaspiMouse2::leds_command, this, _1));

  power_control_ = std::make_shared<std::ofstream>("/dev/rtmotoren0");
  if (!power_control_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open motor power device /dev/rtmotoren0");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }
  left_motor_control_ = std::make_shared<std::ofstream>("/dev/rtmotor_raw_l0");
  if (!left_motor_control_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open left motor device /dev/rtmotor_raw_l0");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }
  right_motor_control_ = std::make_shared<std::ofstream>("/dev/rtmotor_raw_r0");
  if (!right_motor_control_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open right motor device /dev/rtmotor_raw_r0");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }
  led0_output_ = std::make_shared<std::ofstream>("/dev/rtled0");
  if (!led0_output_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open LED 0 device /dev/rtled0");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }
  led1_output_ = std::make_shared<std::ofstream>("/dev/rtled1");
  if (!led1_output_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open LED 1 device /dev/rtled1");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }
  led2_output_ = std::make_shared<std::ofstream>("/dev/rtled2");
  if (!led2_output_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open LED 2 device /dev/rtled2");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }
  led3_output_ = std::make_shared<std::ofstream>("/dev/rtled3");
  if (!led3_output_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open LED 3 device /dev/rtled3");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }
  switch0_input_ = std::make_shared<std::ofstream>("/dev/rtswitch0");
  if (!switch0_input_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open switch 0 device /dev/rtswitch0");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }
  switch1_input_ = std::make_shared<std::ofstream>("/dev/rtswitch1");
  if (!switch1_input_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open switch 1 device /dev/rtswitch1");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }
  switch2_input_ = std::make_shared<std::ofstream>("/dev/rtswitch2");
  if (!switch2_input_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open switch 2 device /dev/rtswitch2");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }
  light_sensors_input_ = std::make_shared<std::ofstream>("/dev/rtlightsensor0");
  if (!light_sensors_intput_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open light sensors device /dev/rtlightsensor0");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE;
  }

  // Disable motors at startup
  // This will also disable the watchdog timer until it is needed (i.e. the motors are
  // turned on)
  set_motor_power(false);

  RCLCPP_INFO(this->get_logger(), "Configuring done");
  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

rcl_lifecycle_transition_key_t RaspiMouse2::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating Raspimouse node");

  odom_pub_->on_activate();
  switches_pub_->on_activate();
  light_sensors_pub_->on_activate();
  // Start the odometry calculations and sensor publishing
  odom_timer_->reset();
  sensors_timer_->reset();

  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

rcl_lifecycle_transition_key_t RaspiMouse2::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating node");

  // Power off the motors
  set_motor_power(false);

  // Stop the odometry calculations and sensor publishing
  odom_timer_->cancel();
  odom_pub_->on_deactivate();
  sensors_timer_->cancel();
  switches_pub_->on_deactivate();
  light_sensors_pub_->on_deactivate();

  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

rcl_lifecycle_transition_key_t RaspiMouse2::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up node");

  odom_pub_.reset();
  odom_transform_broadcaster_.reset();
  odom_timer_.reset();
  velocity_sub_.reset();
  power_service_.reset();
  watchdog_timer_.reset();

  switches_pub_.reset();
  light_sensors_pub_.reset();
  leds_sub_.reset();
  sensors_timer_.reset();

  power_control_.reset();
  left_motor_control_.reset();
  right_motor_control_.reset();
  led0_output_.reset();
  led1_output_.reset();
  led2_output_.reset();
  led3_output_.reset();
  switch0_input_.reset();
  switch1_input_.reset();
  switch2_input_.reset();
  light_sensors_input_.reset();

  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

void RaspiMouse2::publish_odometry()
{
  auto old_last_odom_time = last_odom_time_;
  last_odom_time_ = now();
  auto dt = last_odom_time_ - old_last_odom_time;

  odom_.pose.pose.position.x += linear_velocity_ * cos(odom_theta_) * dt.nanoseconds() / 1e9;
  odom_.pose.pose.position.y += linear_velocity_ * sin(odom_theta_) * dt.nanoseconds() / 1e9;
  odom_theta_ += angular_velocity_ * dt.nanoseconds() / 1e9;
  tf2::Quaternion odom_q;
  odom_q.setRPY(0, 0, odom_theta_);
  odom_.pose.pose.orientation.x = odom_q.x();
  odom_.pose.pose.orientation.y = odom_q.y();
  odom_.pose.pose.orientation.z = odom_q.z();
  odom_.pose.pose.orientation.w = odom_q.w();
  odom_.twist.twist.linear.x = linear_velocity_;
  odom_.twist.twist.angular.z = angular_velocity_;
  odom_pub_->publish(odom_);

  /* TODO: When the tf2 API is updated to match rclcpp, re-enable this broadcaster
  odom_transform_.header.stamp = last_odom_time_;
  odom_transform_.transform.translation.x = odom_.pose.pose.position.x;
  odom_transform_.transform.translation.y = odom_.pose.pose.position.y;
  odom_transform_.transform.rotation.x = odom_q.x();
  odom_transform_.transform.rotation.y = odom_q.y();
  odom_transform_.transform.rotation.z = odom_q.z();
  odom_transform_.transform.rotation.w = odom_q.w();
  odom_transform_broadcaster_->sendTransform(odom_transform_);*/
}

void RaspiMouse2::publish_sensors()
{
  publish_switches();
  publish_light_sensors();
}

void RaspiMouse2::publish_switches()
{
  raspimouse2::msg::Switches switch_states;
  switch_states.switch0 = false;
  switch_states.switch1 = false;
  switch_states.switch2 = false;
  switches_pub_->publish(switch_states);
}

void RaspiMouse2::publish_light_sensors()
{
}

void RaspiMouse2::velocity_command(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  linear_velocity_ = msg->linear.x;
  angular_velocity_ = msg->angular.z;

  auto forward_hz = 80000 * linear_velocity_ / (9 * M_PI);
  auto rotation_hz = 400 * angular_velocity_ / M_PI;

  *left_motor_control_ << static_cast<int>(round(forward_hz - rotation_hz)) << std::endl;
  *right_motor_control_ << static_cast<int>(round(forward_hz + rotation_hz)) << std::endl;
  // Reset the watchdog timeout
  watchdog_timer_->reset();
}

void RaspiMouse2::leds_command(const raspimouse2::msg::LEDs::SharedPtr msg)
{
  if (msg->led0) {
    *led0_output_ << 1;
  } else {
    *led0_output_ << 0;
  }

  if (msg->led1) {
    *led1_output_ << 1;
  } else {
    *led1_output_ << 0;
  }

  if (msg->led2) {
    *led2_output_ << 1;
  } else {
    *led2_output_ << 0;
  }

  if (msg->led3) {
    *led3_output_ << 1;
  } else {
    *led3_output_ << 0;
  }
}

void RaspiMouse2::handle_motor_power(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request_header;
  set_motor_power(request->data);
  response->success = true;
  if (request->data) {
    response->message = "Motors are on";
  } else {
    response->message = "Motors are off";
  }
}

void RaspiMouse2::watchdog()
{
  RCLCPP_INFO(get_logger(), "Watchdog timeout; stopping motors");
  stop_motors();
  // Stop the watchdog timer
  watchdog_timer_->cancel();
}

void RaspiMouse2::set_motor_power(bool value)
{
  if (value) {
    *power_control_ << '1' << std::endl;
    RCLCPP_INFO(get_logger(), "Turned motors on");
    // Start the watchdog timer
    watchdog_timer_->reset();
  } else {
    *power_control_ << '0' << std::endl;
    RCLCPP_INFO(get_logger(), "Turned motors off");
    // Stop the watchdog timer
    watchdog_timer_->cancel();
    // Set motor speeds to zero to prevent surprises
    stop_motors();
  }
}

void RaspiMouse2::stop_motors()
{
  *left_motor_control_ << 0 << std::endl;
  *right_motor_control_ << 0 << std::endl;
}

} // namespace raspimouse2

#include <class_loader/register_macro.hpp>

CLASS_LOADER_REGISTER_CLASS(raspimouse2::RaspiMouse2, rclcpp_lifecycle::LifecycleNode)
