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

#include "raspimouse/raspimouse_component.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
#include <chrono>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rosidl_generator_cpp/message_initialization.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

constexpr auto use_pulse_counters_param = "use_pulse_counters";
constexpr auto use_light_sensors_param = "use_light_sensors";
constexpr auto odometry_scale_left_wheel_param = "odometry_scale_left_wheel";
constexpr auto odometry_scale_right_wheel_param = "odometry_scale_right_wheel";

constexpr auto wheel_diameter = 0.048;
constexpr auto wheel_base = 0.09;

namespace raspimouse
{

Raspimouse::Raspimouse(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("raspimouse", options),
  ros_clock_(RCL_ROS_TIME),
  odom_(rosidl_generator_cpp::MessageInitialization::ZERO),
  odom_transform_(rosidl_generator_cpp::MessageInitialization::ZERO),
  last_odom_time_(0),
  linear_velocity_(0),
  angular_velocity_(0),
  odom_theta_(0),
  use_pulse_counters_(false),
  last_pulse_count_left_(0),
  last_pulse_count_right_(0)
{
  // No construction necessary (node is uninitialised)
}

CallbackReturn Raspimouse::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring Raspimouse node");

  using namespace std::placeholders;  // for _1, _2, _3...

  linear_velocity_ = 0;
  angular_velocity_ = 0;
  last_odom_time_ = now();

  // Publisher for odometry data
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_footprint";
  odom_.pose.pose.position.x = 0;
  odom_.pose.pose.position.y = 0;
  odom_.pose.pose.orientation.x = 0;
  odom_.pose.pose.orientation.y = 0;
  odom_.pose.pose.orientation.z = 0;
  odom_.pose.pose.orientation.w = 0;
  odom_theta_ = 0;
  // Publisher for odometry transform
  odom_transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
      this->shared_from_this());
  odom_transform_.header.frame_id = "odom";
  odom_transform_.child_frame_id = "base_footprint";
  odom_transform_.transform.translation.x = 0;
  odom_transform_.transform.translation.y = 0;
  odom_transform_.transform.rotation.x = 0;
  odom_transform_.transform.rotation.y = 0;
  odom_transform_.transform.rotation.z = 0;
  odom_transform_.transform.rotation.w = 0;
  // Timer for providing the odometry data
  odom_timer_ = create_wall_timer(100ms, std::bind(&Raspimouse::publish_odometry, this));
  // Don't actually start publishing odometry data until activated
  odom_timer_->cancel();

  // Subscriber for velocity commands
  velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&Raspimouse::velocity_command, this, _1));

  // Motor power control service
  power_service_ = create_service<std_srvs::srv::SetBool>(
      "motor_power", std::bind(&Raspimouse::handle_motor_power, this, _1, _2, _3));

  // Watchdog timer to prevent out-of-control robots
  watchdog_timer_ = create_wall_timer(60s, std::bind(&Raspimouse::watchdog, this));

  // Publisher for switch states
  switches_pub_ = this->create_publisher<raspimouse_msgs::msg::Switches>("switches", 10);
  // Publisher for light sensors
  light_sensors_pub_ = this->create_publisher<raspimouse_msgs::msg::LightSensors>(
    "light_sensors", 10);
  // Timer for publishing switch information
  switches_timer_ = create_wall_timer(100ms, std::bind(
    &Raspimouse::publish_switches, this));
  switches_timer_->cancel();
  // Timer for publishing light sensor information
  light_sensors_timer_ = create_wall_timer(100ms, std::bind(
    &Raspimouse::publish_light_sensors, this));
  light_sensors_timer_->cancel();
  // Subscriber for LED commands
  leds_sub_ = create_subscription<raspimouse_msgs::msg::Leds>(
      "leds", 10, std::bind(&Raspimouse::leds_command, this, _1));
  // Subscriber for buzzer commands
  buzzer_sub_ = create_subscription<std_msgs::msg::Int16>(
      "buzzer", 10, std::bind(&Raspimouse::buzzer_command, this, _1));

  power_control_ = std::make_shared<std::ofstream>("/dev/rtmotoren0");
  if (!power_control_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open motor power device /dev/rtmotoren0");
    return CallbackReturn::FAILURE;
  }
  left_motor_control_ = std::make_shared<std::ofstream>("/dev/rtmotor_raw_l0");
  if (!left_motor_control_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open left motor device /dev/rtmotor_raw_l0");
    return CallbackReturn::FAILURE;
  }
  right_motor_control_ = std::make_shared<std::ofstream>("/dev/rtmotor_raw_r0");
  if (!right_motor_control_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open right motor device /dev/rtmotor_raw_r0");
    return CallbackReturn::FAILURE;
  }
  led0_output_ = std::make_shared<std::ofstream>("/dev/rtled0");
  if (!led0_output_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open LED 0 device /dev/rtled0");
    return CallbackReturn::FAILURE;
  }
  led1_output_ = std::make_shared<std::ofstream>("/dev/rtled1");
  if (!led1_output_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open LED 1 device /dev/rtled1");
    return CallbackReturn::FAILURE;
  }
  led2_output_ = std::make_shared<std::ofstream>("/dev/rtled2");
  if (!led2_output_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open LED 2 device /dev/rtled2");
    return CallbackReturn::FAILURE;
  }
  led3_output_ = std::make_shared<std::ofstream>("/dev/rtled3");
  if (!led3_output_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open LED 3 device /dev/rtled3");
    return CallbackReturn::FAILURE;
  }
  buzzer_output_ = std::make_shared<std::ofstream>("/dev/rtbuzzer0");
  if (!buzzer_output_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open buzzer device /dev/rtbuzzer0");
    return CallbackReturn::FAILURE;
  }

  // Disable motors at startup
  // This will also disable the watchdog timer until it is needed (i.e. the motors are
  // turned on)
  set_motor_power(false);

  // Set parameter defaults
  declare_parameter(use_pulse_counters_param, false);
  declare_parameter(use_light_sensors_param, true);
  declare_parameter(odometry_scale_left_wheel_param, 1.0);
  declare_parameter(odometry_scale_right_wheel_param, 1.0);

  // Test if the pulse counters are available
  if (get_parameter(use_pulse_counters_param).get_value<bool>()) {
    RCLCPP_INFO(get_logger(), "Testing counters");
    std::ifstream left_counter("/dev/rtcounter_l0");
    std::ifstream right_counter("/dev/rtcounter_r0");
    if (left_counter.is_open() && right_counter.is_open()) {
      RCLCPP_INFO(get_logger(), "Using pulse counters for odometry");
      use_pulse_counters_ = true;
      left_counter >> last_pulse_count_left_;
      right_counter >> last_pulse_count_right_;
    } else {
      RCLCPP_INFO(get_logger(), "Pulse counters not found; using estimated odometry");
    }
  } else {
    RCLCPP_INFO(get_logger(), "Pulse counters disabled by parameter; using estimated odometry");
  }

  RCLCPP_INFO(this->get_logger(), "Configuring done");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Raspimouse::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating Raspimouse node");

  odom_pub_->on_activate();
  switches_pub_->on_activate();
  light_sensors_pub_->on_activate();
  // Start the odometry calculations and sensor publishing
  odom_timer_->reset();
  switches_timer_->reset();

  // Only enable the light sensors timer if the light sensors parameter is true
  if (get_parameter(use_light_sensors_param).get_value<bool>()) {
    light_sensors_timer_->reset();
  }

  RCLCPP_INFO(this->get_logger(), "Raspimouse node activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Raspimouse::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating node");

  // Power off the motors
  set_motor_power(false);

  // Stop the odometry calculations and sensor publishing
  odom_timer_->cancel();
  odom_pub_->on_deactivate();
  switches_timer_->cancel();
  light_sensors_timer_->cancel();
  switches_pub_->on_deactivate();
  light_sensors_pub_->on_deactivate();

  // For the sake of peace and quiet, stop the buzzer
  *buzzer_output_ << 0 << std::endl;

  return CallbackReturn::SUCCESS;
}

CallbackReturn Raspimouse::on_cleanup(const rclcpp_lifecycle::State &)
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
  buzzer_sub_.reset();
  switches_timer_.reset();
  light_sensors_timer_.reset();

  power_control_.reset();
  left_motor_control_.reset();
  right_motor_control_.reset();
  led0_output_.reset();
  led1_output_.reset();
  led2_output_.reset();
  led3_output_.reset();
  buzzer_output_.reset();

  return CallbackReturn::SUCCESS;
}

void Raspimouse::publish_odometry()
{
  if (use_pulse_counters_) {
    calculate_odometry_from_pulse_counts(
      odom_.pose.pose.position.x,
      odom_.pose.pose.position.y,
      odom_theta_
      );
  } else {
    estimate_odometry(
      odom_.pose.pose.position.x,
      odom_.pose.pose.position.y,
      odom_theta_
      );
  }

  tf2::Quaternion odom_q;
  odom_q.setRPY(0, 0, odom_theta_);
  odom_.pose.pose.orientation.x = odom_q.x();
  odom_.pose.pose.orientation.y = odom_q.y();
  odom_.pose.pose.orientation.z = odom_q.z();
  odom_.pose.pose.orientation.w = odom_q.w();
  odom_.twist.twist.linear.x = linear_velocity_;
  odom_.twist.twist.angular.z = angular_velocity_;
  odom_.header.stamp = ros_clock_.now();
  odom_pub_->publish(odom_);

  odom_transform_.header.stamp = last_odom_time_;
  odom_transform_.transform.translation.x = odom_.pose.pose.position.x;
  odom_transform_.transform.translation.y = odom_.pose.pose.position.y;
  odom_transform_.transform.rotation.x = odom_q.x();
  odom_transform_.transform.rotation.y = odom_q.y();
  odom_transform_.transform.rotation.z = odom_q.z();
  odom_transform_.transform.rotation.w = odom_q.w();
  odom_transform_broadcaster_->sendTransform(odom_transform_);
}

void Raspimouse::publish_switches()
{
  std::ifstream switch0_input("/dev/rtswitch0");
  if (!switch0_input.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open switch 0 device /dev/rtswitch0");
    // TODO(ShotaAk): Error state transition
    return;
  }
  std::ifstream switch1_input("/dev/rtswitch1");
  if (!switch1_input.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open switch 1 device /dev/rtswitch1");
    // TODO(ShotaAk): Error state transition
    return;
  }
  std::ifstream switch2_input("/dev/rtswitch2");
  if (!switch2_input.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open switch 2 device /dev/rtswitch2");
    // TODO(ShotaAk): Error state transition
    return;
  }
  raspimouse_msgs::msg::Switches switch_states;
  char c;
  switch0_input >> c;
  if (c == '0') {
    switch_states.switch0 = true;
  } else {
    switch_states.switch0 = false;
  }
  switch1_input >> c;
  if (c == '0') {
    switch_states.switch1 = true;
  } else {
    switch_states.switch1 = false;
  }
  switch2_input >> c;
  if (c == '0') {
    switch_states.switch2 = true;
  } else {
    switch_states.switch2 = false;
  }
  switches_pub_->publish(switch_states);
}

void Raspimouse::publish_light_sensors()
{
  std::ifstream light_sensors_input("/dev/rtlightsensor0");
  if (!light_sensors_input.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open light sensors device /dev/rtlightsensor0");
    // TODO(ShotaAk): Error state transition
    return;
  }
  raspimouse_msgs::msg::LightSensors sensor_values;
  light_sensors_input >> sensor_values.forward_r
    >> sensor_values.right
    >> sensor_values.left
    >> sensor_values.forward_l;
  light_sensors_pub_->publish(sensor_values);
}

void Raspimouse::velocity_command(const geometry_msgs::msg::Twist::SharedPtr msg)
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

void Raspimouse::leds_command(const raspimouse_msgs::msg::Leds::SharedPtr msg)
{
  if (msg->led0) {
    *led0_output_ << 1 << std::endl;
  } else {
    *led0_output_ << 0 << std::endl;
  }

  if (msg->led1) {
    *led1_output_ << 1 << std::endl;
  } else {
    *led1_output_ << 0 << std::endl;
  }

  if (msg->led2) {
    *led2_output_ << 1 << std::endl;
  } else {
    *led2_output_ << 0 << std::endl;
  }

  if (msg->led3) {
    *led3_output_ << 1 << std::endl;
  } else {
    *led3_output_ << 0 << std::endl;
  }
}

void Raspimouse::buzzer_command(const std_msgs::msg::Int16::SharedPtr msg)
{
  *buzzer_output_ << msg->data << std::endl;
}

void Raspimouse::handle_motor_power(
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

void Raspimouse::watchdog()
{
  RCLCPP_INFO(get_logger(), "Watchdog timeout; stopping motors");
  stop_motors();
  // Stop the watchdog timer
  watchdog_timer_->cancel();
}

void Raspimouse::set_motor_power(bool value)
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

void Raspimouse::stop_motors()
{
  *left_motor_control_ << 0 << std::endl;
  *right_motor_control_ << 0 << std::endl;
}

void Raspimouse::calculate_odometry_from_pulse_counts(double &x, double &y, double &theta)
{
  auto one_revolution_distance_left = 2 * M_PI * wheel_diameter *
    get_parameter(odometry_scale_left_wheel_param).get_value<double>();
  auto one_revolution_distance_right = 2 * M_PI * wheel_diameter *
    get_parameter(odometry_scale_right_wheel_param).get_value<double>();

  RCLCPP_INFO(get_logger(), "Reading counters");
  std::ifstream left_counter("/dev/rtcounter_l0");
  std::ifstream right_counter("/dev/rtcounter_r0");
  int pulse_count_left, pulse_count_right;
  left_counter >> pulse_count_left;
  right_counter >> pulse_count_right;
  RCLCPP_INFO(get_logger(), "Old: %d, %d\tNew: %d, %d", last_pulse_count_left_,
  last_pulse_count_right_, pulse_count_left, pulse_count_right);

  // Account for rollover
  int pulse_count_difference_left(0), pulse_count_difference_right(0);
  if (pulse_count_left < last_pulse_count_left_) {
    pulse_count_difference_left = (65535 - last_pulse_count_left_) + pulse_count_left;
  } else {
    pulse_count_difference_left = pulse_count_left - last_pulse_count_left_;
  }
  if (pulse_count_right < last_pulse_count_right_) {
    pulse_count_difference_right = (65535 - last_pulse_count_right_) + pulse_count_right;
  } else {
    pulse_count_difference_right = pulse_count_right - last_pulse_count_right_;
  }
  RCLCPP_INFO(get_logger(), "Pulse differences: %d, %d", pulse_count_difference_left,
  pulse_count_difference_right);

  // Calculate number of revolutions since last time
  // 400 pulses per revolution
  auto left_revolutions = pulse_count_difference_left / 400.0;
  auto right_revolutions = pulse_count_difference_right / 400.0;
  RCLCPP_INFO(get_logger(), "Revolutions: %f, %f", left_revolutions, right_revolutions);
  // Calculate the distance the wheel has travelled (ignoring slip)
  auto left_distance = left_revolutions * one_revolution_distance_left;
  auto right_distance = right_revolutions * one_revolution_distance_right;
  auto average_distance = (right_distance - left_distance) / 2;
  RCLCPP_INFO(get_logger(), "Left dist: %f\tRight dist: %f\tAverage: %f",
  left_distance, right_distance, average_distance);

  last_pulse_count_left_ = pulse_count_left;
  last_pulse_count_right_ = pulse_count_right;

  theta += atan2(right_distance - left_distance, wheel_base);
  x += average_distance * cos(theta);
  y += average_distance * sin(theta);

  RCLCPP_INFO(get_logger(), "Counter: x: %f\ty: %f\ttheta: %f", x, y, theta);

  last_odom_time_ = now();
}

void Raspimouse::estimate_odometry(double &x, double &y, double &theta)
{
  auto old_last_odom_time = last_odom_time_;
  last_odom_time_ = now();
  auto dt = last_odom_time_ - old_last_odom_time;

  x += linear_velocity_ * cos(theta) * dt.nanoseconds() / 1e9;
  y += linear_velocity_ * sin(theta) * dt.nanoseconds() / 1e9;
  theta += angular_velocity_ * dt.nanoseconds() / 1e9;
}

}  // namespace raspimouse

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(raspimouse::Raspimouse)
