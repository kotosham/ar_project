cmake_minimum_required(VERSION 3.5)
project(ar_project)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(canopen_402_driver REQUIRED)
find_package(canopen_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(ar_project_embodied_robot_system SHARED
  src/embodied_robot_system.cpp
)

target_include_directories(ar_project_embodied_robot_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ar_project_embodied_robot_system
  canopen_402_driver
  canopen_ros2_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface ar_project_hardware_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
    scripts/calibration_motion.py
    scripts/camera_info_relay.py
    scripts/cmd_vel_watchdog.py
    scripts/experiment_metrics_logger.py
    scripts/home_pose_manager.py
    scripts/odom_to_tf.py
    scripts/tracker_rgb_bridge.py
    scripts/tracker_rgbd_bridge.py
    scripts/twist_to_twist_stamped.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ar_project_embodied_robot_system
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

cogen_dcf(epos4_diffdrive)

ament_package()
