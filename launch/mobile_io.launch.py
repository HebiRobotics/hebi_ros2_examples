import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, LogInfo
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals


def generate_launch_description():

  # Declare arguments
  declared_arguments = []
  declared_arguments.append(
      DeclareLaunchArgument(
          "cfg_file",
          default_value="mobile_io.cfg.yaml",
          description="Config file for the mobile_io of type `.cfg.yaml`",
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "description_package",
          default_value="hebi_description",
          description="Description package of the A-2085-06. Usually the argument is not set, \
                       it enables use of a custom description.",
      )
  )

  # Set default values for arguments
  default_arguments = []
  default_arguments.append(
    LogInfo(
      msg=PythonExpression(['"Using default config file: ', 'mobile_io.cfg.yaml"']),
      condition=LaunchConfigurationEquals("cfg_file", "None")
    )
  )
  default_arguments.append(
    SetLaunchConfiguration(
      name="cfg_file",
      value=PythonExpression(['"', LaunchConfiguration("hebi_arm"), '_params.yaml"']),
      condition=LaunchConfigurationEquals("params_file", "None")
    )
  )
  
  cfg_file = LaunchConfiguration("cfg_file")
  description_package = LaunchConfiguration("description_package")

  # Get URDF via xacro
  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          PathJoinSubstitution(
              [FindPackageShare(description_package), "urdf", "kits", PythonExpression(['"', hebi_arm, '.urdf.xacro"'])]
          ),
          " "
          "prefix:=",
          prefix,
      ]
  )

  robot_description = {"robot_description": robot_description_content}
  robot_state_publisher_node = Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      output="both",
      parameters=[robot_description],
      namespace=prefix,
      remappings=[('/joint_states', '/fdbk_joint_states')]
  )

  robot_params = PathJoinSubstitution(
    [FindPackageShare('hebi_ros2_examples'), 'config', params_file]
  )
  arm_node = Node(
    package='hebi_ros2_examples',
    executable='arm_node',
    name='arm_node',
    output='screen',
    parameters=[
      robot_params,
      {"prefix": prefix}
    ],
    namespace=prefix,
  )

  mobile_io_config = PathJoinSubstitution(
    [FindPackageShare('hebi_ros2_examples'), 'config', cfg_file]
  )

  mobile_io_params = 

  mobile_io_node = Node(
    package='hebi_ros2_examples',
    executable='mobile_io_node',
    name='mobile_io_node',
    output='screen',
    parameters=[
      mobile_io_params,
      {"prefix": prefix}
    ],
    namespace=prefix,
  )

  
  rviz_config_file = PathJoinSubstitution(
    [FindPackageShare(description_package), "rviz", "hebi_arm.rviz"]
  )
  rviz_node = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      output="log",
      arguments=["-d", rviz_config_file],
      condition=IfCondition(LaunchConfiguration("use_rviz")),
  )

  return LaunchDescription(
    declared_arguments +
    default_arguments +
    [
      robot_state_publisher_node,
      arm_node,
      rviz_node
    ]
  )