cmake_minimum_required(VERSION 3.20)
project(raspimouse)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(raspimouse_msgs REQUIRED)

add_library(raspimouse_component SHARED src/raspimouse_component.cpp)
target_compile_definitions(raspimouse_component PRIVATE "RASPIMOUSE_BUILDING_DLL")
target_include_directories(raspimouse_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(raspimouse_component PUBLIC
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  raspimouse_msgs::raspimouse_msgs
  std_msgs::std_msgs
  std_srvs::std_srvs
  tf2::tf2
  tf2_ros::tf2_ros)
rclcpp_components_register_nodes(raspimouse_component "raspimouse::Raspimouse")

add_executable(raspimouse src/raspimouse.cpp)
target_include_directories(raspimouse PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(raspimouse PRIVATE
  class_loader::class_loader
  raspimouse_component)

ament_export_targets(export_raspimouse_component HAS_LIBRARY_TARGET)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_lifecycle)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(raspimouse_msgs)

ament_export_include_directories(include)
ament_export_libraries(raspimouse_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS raspimouse_component
  EXPORT export_raspimouse_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(TARGETS
  raspimouse
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
