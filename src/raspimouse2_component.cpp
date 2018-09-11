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
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <class_loader/class_loader_register_macro.h>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

using namespace std::chrono_literals;

namespace raspimouse2
{

RaspiMouse2::RaspiMouse2()
: rclcpp_lifecycle::LifecycleNode("raspimouse2"),
  odom_(rosidl_generator_cpp::MessageInitialization::ZERO),
  odom_transform_(rosidl_generator_cpp::MessageInitialization::ZERO),
  linear_velocity(0),
  angular_velocity(0),
  last_odom_time_(0)
{
  // No construction necessary (node is uninitialised)
}

rcl_lifecycle_transition_key_t RaspiMouse2::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring node");

  // Publisher for odometry data
  odom_pub_ = create_publisher<nav_msgs::Odometry>("odom");
  odom_.pose.pose.position.x = 0;
  odom_.pose.pose.position.y = 0;
  odom_.pose.pose.orientation.x = 0;
  odom_.pose.pose.orientation.y = 0;
  odom_.pose.pose.orientation.z = 0;
  odom_.pose.pose.orientation.w = 0;
  odom_theta_ = 0;
  // Publisher for odometry transform
  odom_transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
  odom_transform_.header.frame_id = "odom";
  odom_transform_.header.child_frame_id = "base_link";
  odom_transform_.transform.translation.x = 0;
  odom_transform_.transform.translation.y = 0;
  odom_transform_.transform.rotation.x = 0;
  odom_transform_.transform.rotation.y = 0;
  odom_transform_.transform.rotation.z = 0;
  odom_transform_.transform.rotation.w = 0;
  // Timer for providing the odometry data
  odom_timer_ = create_wall_timer(0.1s, std::bind(&RaspiMouse2::publish_odometry, this));

  // Subscriber for velocity commands
  velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", std::bind(&RaspiMouse2::msg::velocity_command, this, _1));

  // Motor power control service
  power_service_ = create_service<std_srvs::srv::SetBool>(
      "motor_power", std::bind(&RaspiMouse2::handle_motor_power, this, _1, _2, _3));

  // Watchdog timer to prevent out-of-control robots
  watchdog_timer_ = create_wall_timer(5s, std::bind(&RaspiMouse2::watchdog, this));

  power_control_ = std::make_shared<std::ofstream>("/dev/rtmotoren0");
  if (!power_control_.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open motor power device /dev/rtmotoren0");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILED;
  }
  left_motor_control_ = std::make_shared<std::ofstream>("/dev/rtmotor_raw_l0");
  if (!left_motor_control_.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open left motor device /dev/rtmotor_raw_l0");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILED;
  }
  right_motor_control_ = std::make_shared<std::ofstream>("/dev/rtmotor_raw_r0");
  if (!right_motor_control_.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open right motor device /dev/rtmotor_raw_r0");
    return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILED;
  }

  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

rcl_lifecycle_transition_key_t RaspiMouse2::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating node");

  odom_pub_->on_activate();
  velocity_sub_->on_activate();
  power_service_->on_activate();

  linear_velocity = 0;
  angular_velocity = 0;
  last_odom_time_ = now();

  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

rcl_lifecycle_transition_key_t RaspiMouse2::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating node");

  // Power off
  set_motor_power(false);

  odom_pub_->on_deactivate();
  velocity_sub_->on_deactivate();
  power_service_->on_deactivate();

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

  power_control_.reset();
  left_motor_control_.reset();
  right_motor_control_.reset();

  return lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS;
}

} // namespace raspimouse2

void RaspiMouse2::publish_odometry()
{
  auto old_last_odom_time = last_odom_time_;
  last_odom_time_ = now();
  auto dt = last_odom_time_ - old_last_odom_time_;

  odom_.pose.pose.position.x += linear_velocity * cos(odom_theta) * dt.seconds();
  odom_.pose.pose.position.y += linear_velocity * sin(odom_theta) * dt.seconds();
  odom_theta_ += angular.z += angular_velocity * dt.seconds();
  odom_q_.setRPY(0, 0, odom_theta_);
  odom_.pose.orientation = odom_q_;
  odom_pub_.publish(odom_);

  odom_transform_.header.stamp = last_odom_time_;
  odom_transform_.transform.translation.x = odom_.pose.pose.position.x;
  odom_transform_.transform.translation.y = odom_.pose.pose.position.y;
  odom_transform_.transform.rotation.x = odom_q.x();
  odom_transform_.transform.rotation.y = odom_q.y();
  odom_transform_.transform.rotation.z = odom_q.z();
  odom_transform_.transform.rotation.w = odom_q.w();
  odom_transform_broadcaster_.sendTransform(odom_transform_);
}

void RaspiMouse2::velocity_command(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  linear_velocity = msg->linear.x;
  angular_velocity = msg->angular.z;

  auto forward_hz = 80000 * msg->linear.x / (9 * M_PI);
  auto rotation_hz = 400 * msg->angular.z / M_PI;

  left_motor_control_ << static_cast<int>(round(forward_hz - rotation_hz)) << std::endl;
  right_motor_control_ << static_cast<int>(round(forward_hz + rotation_hz)) << std::endl;
  watchdog_timer_->reset();
}

void RaspiMouse2::handle_motor_power(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request_header;
  set_power(request->data);
  response->success = true;
  if (request->data) {
    response->message = "Motors are on";
  } else {
    response->message = "Motors are off";
  }
}

void watchdog()
{
  RCLCPP_INFO(get_logger(), "Watchdog timeout; stopping motors");
  stop_motors();
}

void RaspiMouse2::set_motor_power(bool value)
{
  if (value) {
    ofs << '1' << std::endl;
    RCLCPP_INFO(get_logger(), "Turned motors on");
    // Start the watchdog timer
    watchdog_timer_->reset();
  } else {
    ofs << '0' << std::endl;
    RCLCPP_INFO(get_logger(), "Turned motors off");
  }
}

void RaspiMouse2::stop_motors()
{
  left_motor_control_ << 0 << std::endl;
  right_motor_control_ << 0 << std::endl;
}

CLASS_LOADER_REGISTER_CLASS(raspimouse2::RaspiMouse2, rclcpp_lifecycle::LifecycleNode)
