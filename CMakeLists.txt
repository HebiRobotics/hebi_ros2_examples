cmake_minimum_required(VERSION 3.8)
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
find_package(Eigen3)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hebi_cpp_api REQUIRED)
find_package(hebi_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(action_msgs REQUIRED)
# find_package(control_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)

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
)

install(TARGETS
  arm_node
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
