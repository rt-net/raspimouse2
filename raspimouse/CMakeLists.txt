cmake_minimum_required(VERSION 3.5)
project(raspimouse)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
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

include_directories(include)

add_library(raspimouse_component SHARED src/raspimouse_component.cpp)
target_compile_definitions(raspimouse_component PRIVATE "RASPIMOUSE_BUILDING_DLL")
ament_target_dependencies(raspimouse_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  raspimouse_msgs)
rclcpp_components_register_nodes(raspimouse_component "raspimouse::Raspimouse")

add_executable(raspimouse src/raspimouse.cpp)
target_link_libraries(raspimouse raspimouse_component)
ament_target_dependencies(raspimouse
  rclcpp
  rclcpp_lifecycle
  class_loader
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  raspimouse_msgs)

ament_export_interfaces(export_raspimouse_component HAS_LIBRARY_TARGET)

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
