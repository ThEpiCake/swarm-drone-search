cmake_minimum_required(VERSION 3.8)
project(swarm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
    scripts/start_sitl.sh
    scripts/start_multi_sitl.sh
    scripts/camera_view.py
    scripts/odom_publisher.py
    scripts/slam_pose_fusion.py
    scripts/slam_to_px4.py
    scripts/lidar_tilt_filter.py
    scripts/gz_truth_bridge.py
    scripts/entropy_centroid_publisher.py
    scripts/scan_to_range.py
    scripts/visualize_layers.py
    scripts/map_merger.py
    scripts/lifecycle_autostart.py
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
