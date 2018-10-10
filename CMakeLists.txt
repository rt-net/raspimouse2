cmake_minimum_required(VERSION 3.5)
project(raspimouse2)

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
find_package(class_loader REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_library(raspimouse2_component SHARED src/raspimouse2_component.cpp)
target_compile_definitions(raspimouse2_component PRIVATE "RASPIMOUSE2_BUILDING_DLL")
ament_target_dependencies(raspimouse2_component
  rclcpp
  rclcpp_lifecycle
  class_loader
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  )
rclcpp_register_node_plugins(raspimouse2_component "raspimouse2::RaspiMouse2")

add_executable(raspimouse2 src/raspimouse2.cpp)
target_link_libraries(raspimouse2 raspimouse2_component)
ament_target_dependencies(raspimouse2
  rclcpp
  rclcpp_lifecycle
  class_loader
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  )

install(TARGETS
  raspimouse2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS
  raspimouse2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
