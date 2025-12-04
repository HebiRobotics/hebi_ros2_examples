cmake_minimum_required(VERSION 3.5)
project(hebi_ros2_examples)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
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

find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface QUIET)
find_package(moveit_msgs QUIET)
find_package(moveit_visual_tools QUIET)

include_directories(
  include
)

add_executable(arm_node src/kits/arms/arm_node.cpp)
target_link_libraries(arm_node
  PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${std_srvs_TARGETS}
  ${hebi_cpp_api_LIBRARIES}
  ${hebi_msgs_TARGETS}
  ${control_msgs_TARGETS}
)
target_include_directories(arm_node PUBLIC ${hebi_cpp_api_INCLUDE_DIRS})

add_executable(ex_arm_joint_motion src/kits/arms/ex_arm_joint_motion.cpp)
target_link_libraries(ex_arm_joint_motion
  PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${trajectory_msgs_TARGETS}
  ${hebi_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

add_executable(ex_arm_cartesian_motion src/kits/arms/ex_arm_cartesian_motion.cpp)
target_link_libraries(ex_arm_cartesian_motion
  PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${hebi_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

add_executable(joystick_node src/kits/arms/joystick_node.cpp)
target_link_libraries(joystick_node
  PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${trajectory_msgs_TARGETS}
  ${hebi_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${hebi_cpp_api_LIBRARIES}
  ${control_msgs_TARGETS}
  ${std_srvs_TARGETS}
)
target_include_directories(joystick_node PUBLIC ${hebi_cpp_api_INCLUDE_DIRS})

add_executable(omni_base_node src/kits/bases/omni_base_node.cpp src/kits/bases/util/omni_base.cpp src/kits/bases/util/odom_publisher.cpp)
target_link_libraries(omni_base_node
  PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${std_srvs_TARGETS}
  ${hebi_cpp_api_LIBRARIES}
  ${hebi_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2_ros::static_transform_broadcaster_node
)
target_include_directories(omni_base_node PUBLIC ${hebi_cpp_api_INCLUDE_DIRS})

add_executable(mecanum_base_node src/kits/bases/mecanum_base_node.cpp src/kits/bases/util/mecanum_base.cpp src/kits/bases/util/odom_publisher.cpp)
target_link_libraries(mecanum_base_node
  PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${std_srvs_TARGETS}
  ${hebi_cpp_api_LIBRARIES}
  ${hebi_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2_ros::static_transform_broadcaster_node
)
target_include_directories(mecanum_base_node PUBLIC ${hebi_cpp_api_INCLUDE_DIRS})

add_executable(diff_drive_node src/kits/bases/diff_drive_node.cpp src/kits/bases/util/diff_drive.cpp src/kits/bases/util/odom_publisher.cpp)
target_link_libraries(diff_drive_node
  PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${std_srvs_TARGETS}
  ${hebi_cpp_api_LIBRARIES}
  ${hebi_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2_ros::static_transform_broadcaster_node
)
target_include_directories(diff_drive_node PUBLIC ${hebi_cpp_api_INCLUDE_DIRS})

# add_executable(hexapod_control src/kits/daisy/hexapod_control.cpp
#   src/kits/daisy/util/leg.cpp
#   src/kits/daisy/util/hexapod.cpp
#   src/kits/daisy/util/step.cpp
#   src/kits/daisy/util/hexapod_parameters.cpp
#   src/kits/daisy/xml_util/pugixml.cpp
#   src/kits/daisy/xml_util/xml_helpers.cpp
# )
# ament_target_dependencies(hexapod_control
#   rclcpp
#   ament_index_cpp
#   geometry_msgs
#   hebi_cpp_api
#   sensor_msgs
# )

add_executable(treaded_base_node src/kits/tready/treaded_base_node.cpp src/kits/tready/treaded_base.cpp)
target_link_libraries(treaded_base_node
  PUBLIC
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${hebi_cpp_api_LIBRARIES}
  ${hebi_msgs_TARGETS}
  ${std_srvs_TARGETS}
)
target_include_directories(treaded_base_node PUBLIC ${hebi_cpp_api_INCLUDE_DIRS})


install(TARGETS
  arm_node
  omni_base_node
  mecanum_base_node
  diff_drive_node
  # hexapod_control
  treaded_base_node
  joystick_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ex_arm_joint_motion
  ex_arm_cartesian_motion
  DESTINATION lib/${PROJECT_NAME})

# Python modules
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
  scripts/ex_publish_joint_trajectory.py
  scripts/ex_publish_cartesian_trajectory.py
  scripts/ex_teach_repeat_mobileio.py
  scripts/ex_teleop_mobileio.py
  scripts/ex_haptic_teleop_node.py
  scripts/mobile_io/mobile_io_node.py
  scripts/mobile_io/record_and_play_io.py
  scripts/mobile_io/rosie_io.py
  scripts/mobile_io/hexapod_io.py
  scripts/utils/joint_state_combiner.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python utilities
install(PROGRAMS
  hebi_ros2_examples/hebi_util.py
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
