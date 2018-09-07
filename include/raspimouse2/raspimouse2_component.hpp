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

#ifndef RASPIMOUSE2__RASPIMOUSE2_COMPONENT_HPP_
#define RASPIMOUSE2__RASPIMOUSE2_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RASPIMOUSE2_EXPORT __attribute__ ((dllexport))
    #define RASPIMOUSE2_IMPORT __attribute__ ((dllimport))
  #else
    #define RASPIMOUSE2_EXPORT __declspec(dllexport)
    #define RASPIMOUSE2_IMPORT __declspec(dllimport)
  #endif
  #ifdef RASPIMOUSE2_BUILDING_DLL
    #define RASPIMOUSE2_PUBLIC RASPIMOUSE2_EXPORT
  #else
    #define RASPIMOUSE2_PUBLIC RASPIMOUSE2_IMPORT
  #endif
  #define RASPIMOUSE2_PUBLIC_TYPE RASPIMOUSE2_PUBLIC
  #define RASPIMOUSE2_LOCAL
#else
  #define RASPIMOUSE2_EXPORT __attribute__ ((visibility("default")))
  #define RASPIMOUSE2_IMPORT
  #if __GNUC__ >= 4
    #define RASPIMOUSE2_PUBLIC __attribute__ ((visibility("default")))
    #define RASPIMOUSE2_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RASPIMOUSE2_PUBLIC
    #define RASPIMOUSE2_LOCAL
  #endif
  #define RASPIMOUSE2_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <greeting_msg/msg/greeting.hpp>

namespace raspimouse2
{

class RaspiMouse2 : public rclcpp_lifecycle::LifecycleNode
{
public:
  RASPIMOUSE2_PUBLIC
  RaspiMouse2();

private:
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  nav_msgs::Odometry::SharedPtr odom_;
  rclcpp::TimerBase::SharedPtr odom_timer_;

  rclcpp_lifecycle::LifecycleSubscriber<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;

  rclcpp_lifecycle::LifecycleService<std_srvs::srv::SetBool>::SharedPtr power_service_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::shared_ptr<std::ofstream> power_control_;
  std::shared_ptr<std::ofstream> left_motor_control_;
  std::shared_ptr<std::ofstream> right_motor_control_;
  double linear_velocity;
  double angular_velocity;
  geometry_msgs::msg::Twist odom_;
  rcl_time_point_value_t last_odom_time_;

  rcl_lifecycle_transition_key_t on_configure(const rclcpp_lifecycle::State &);
  rcl_lifecycle_transition_key_t on_activate(const rclcpp_lifecycle::State &);
  rcl_lifecycle_transition_key_t on_deactivate(const rclcpp_lifecycle::State &);
  rcl_lifecycle_transition_key_t on_cleanup(const rclcpp_lifecycle::State &);

  void publish_odometry();
  void velocity_command(const geometry_msgs::msg::Twist::SharedPtr msg);
  void handle_motor_power(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void watchdog();

  void set_motor_power(bool value);
  void stop_motors();
};

} // namespace managed_greeter

#endif // RASPIMOUSE2__RASPIMOUSE2_COMPONENT_HPP_
