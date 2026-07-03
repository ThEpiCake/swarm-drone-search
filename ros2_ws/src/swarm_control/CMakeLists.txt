cmake_minimum_required(VERSION 3.8)
project(swarm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Active node — Stage D single-agent APF search mission controller.
# Archived nodes (offboard_control, apf_controller, lidar_mapper, frontier_planner)
# are kept under src/swarm_control/archive/ and no longer built.
add_executable(search_mission_controller src/search_mission_controller.cpp)
ament_target_dependencies(search_mission_controller
  rclcpp px4_msgs sensor_msgs std_msgs geometry_msgs nav_msgs swarm_msgs visualization_msgs)

install(TARGETS
  search_mission_controller
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
