cmake_minimum_required(VERSION 3.5)
project(hebi_ros2_examples)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hebi_cpp_api REQUIRED)
find_package(hebi_msgs REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)

include_directories(
  include
)

add_executable(arm_node src/kits/arm/arm_node.cpp)
ament_target_dependencies(arm_node
  rclcpp
  rclcpp_action
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  hebi_cpp_api
  hebi_msgs
  control_msgs
)

add_executable(move_arm src/kits/arm/move_arm.cpp)
ament_target_dependencies(move_arm
  rclcpp
  rclcpp_action
  trajectory_msgs
  hebi_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
)

add_executable(moveit_arm src/kits/arm/moveit_example.cpp)
ament_target_dependencies(moveit_arm
  rclcpp
  trajectory_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
)

add_executable(omni_base_node src/kits/base/omni_base_node.cpp src/util/omni_base.cpp src/util/odom_publisher.cpp)
ament_target_dependencies(omni_base_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  hebi_cpp_api
  hebi_msgs
  tf2_ros
)

add_executable(mecanum_base_node src/kits/base/mecanum_base_node.cpp src/util/mecanum_base.cpp src/util/odom_publisher.cpp)
ament_target_dependencies(mecanum_base_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  hebi_cpp_api
  hebi_msgs
  tf2_ros
)

add_executable(diff_drive_node src/kits/base/diff_drive_node.cpp src/util/diff_drive.cpp src/util/odom_publisher.cpp)
ament_target_dependencies(diff_drive_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  hebi_cpp_api
  hebi_msgs
  tf2_ros
)

install(TARGETS
  arm_node
  move_arm
  moveit_arm
  omni_base_node
  mecanum_base_node
  diff_drive_node
  DESTINATION lib/${PROJECT_NAME})

# Python modules
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
  scripts/record_and_play.py
  scripts/publish_trajectory.py
  scripts/mobile_io/rosie.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  include
  DESTINATION include/${PROJECT_NAME}
)

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
