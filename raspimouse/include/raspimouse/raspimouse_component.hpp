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

#ifndef RASPIMOUSE__RASPIMOUSE_COMPONENT_HPP_
#define RASPIMOUSE__RASPIMOUSE_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RASPIMOUSE_EXPORT __attribute__ ((dllexport))
    #define RASPIMOUSE_IMPORT __attribute__ ((dllimport))
  #else
    #define RASPIMOUSE_EXPORT __declspec(dllexport)
    #define RASPIMOUSE_IMPORT __declspec(dllimport)
  #endif
  #ifdef RASPIMOUSE_BUILDING_DLL
    #define RASPIMOUSE_PUBLIC RASPIMOUSE_EXPORT
  #else
    #define RASPIMOUSE_PUBLIC RASPIMOUSE_IMPORT
  #endif
  #define RASPIMOUSE_PUBLIC_TYPE RASPIMOUSE_PUBLIC
  #define RASPIMOUSE_LOCAL
#else
  #define RASPIMOUSE_EXPORT __attribute__ ((visibility("default")))
  #define RASPIMOUSE_IMPORT
  #if __GNUC__ >= 4
    #define RASPIMOUSE_PUBLIC __attribute__ ((visibility("default")))
    #define RASPIMOUSE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RASPIMOUSE_PUBLIC
    #define RASPIMOUSE_LOCAL
  #endif
  #define RASPIMOUSE_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <raspimouse_msgs/msg/leds.hpp>
#include <raspimouse_msgs/msg/switches.hpp>
#include <raspimouse_msgs/msg/light_sensors.hpp>

#include <memory>

namespace raspimouse
{

class Raspimouse : public rclcpp_lifecycle::LifecycleNode
{
public:
  RASPIMOUSE_PUBLIC
  explicit Raspimouse(const rclcpp::NodeOptions & options);

private:
  rclcpp::Clock ros_clock_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>>
  odom_pub_;
  nav_msgs::msg::Odometry odom_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_transform_broadcaster_;
  geometry_msgs::msg::TransformStamped odom_transform_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::Time last_odom_time_;
  double linear_velocity_;
  double angular_velocity_;
  double odom_theta_;
  bool use_pulse_counters_;
  int last_pulse_count_left_;
  int last_pulse_count_right_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr power_service_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::shared_ptr<std::ofstream> power_control_;
  std::shared_ptr<std::ofstream> left_motor_control_;
  std::shared_ptr<std::ofstream> right_motor_control_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      raspimouse_msgs::msg::Switches>> switches_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      raspimouse_msgs::msg::LightSensors>> light_sensors_pub_;
  rclcpp::Subscription<raspimouse_msgs::msg::Leds>::SharedPtr leds_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr buzzer_sub_;
  rclcpp::TimerBase::SharedPtr switches_timer_;
  rclcpp::TimerBase::SharedPtr light_sensors_timer_;

  std::shared_ptr<std::ofstream> led0_output_;
  std::shared_ptr<std::ofstream> led1_output_;
  std::shared_ptr<std::ofstream> led2_output_;
  std::shared_ptr<std::ofstream> led3_output_;
  std::shared_ptr<std::ofstream> buzzer_output_;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

  void publish_odometry();
  void publish_switches();
  void publish_light_sensors();

  void velocity_command(const geometry_msgs::msg::Twist::SharedPtr msg);
  void leds_command(const raspimouse_msgs::msg::Leds::SharedPtr msg);
  void buzzer_command(const std_msgs::msg::Int16::SharedPtr msg);

  void handle_motor_power(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void watchdog();
  void set_motor_power(bool value);
  void stop_motors();
  void calculate_odometry_from_pulse_counts(double & x, double & y, double & theta);
  void estimate_odometry(double & x, double & y, double & theta);
};

}  // namespace raspimouse

#endif  // RASPIMOUSE__RASPIMOUSE_COMPONENT_HPP_
