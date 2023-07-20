import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  # Declare arguments
  declared_arguments = []
  declared_arguments.append(
      DeclareLaunchArgument(
          "hebi_arm",
          description="Name of the robot to be used.",
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "prefix",
          default_value="",
          description="Prefix for the HEBI Arm name. Usually the argument is not set",
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
  
  hebi_arm = LaunchConfiguration("hebi_arm")
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
          LaunchConfiguration("prefix"),
      ]
  )

  robot_description = {"robot_description": robot_description_content}

  rviz_config_file = PathJoinSubstitution(
    [FindPackageShare(description_package), "rviz", "hebi_arm.rviz"]
  )

  robot_state_publisher_node = Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      output="both",
      parameters=[robot_description],
  )
  rviz_node = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      output="log",
      arguments=["-d", rviz_config_file],
  )

  robot_params = PathJoinSubstitution(
    [FindPackageShare('hebi_ros2_examples'), 'config', PythonExpression(['"', hebi_arm, '_params.yaml"'])]
  )
  arm_node = Node(
    package='hebi_ros2_examples',
    executable='arm_node',
    name='arm_node',
    output='screen',
    parameters=[
      robot_params
    ]
  )

  return LaunchDescription(
    declared_arguments +
    [
      robot_state_publisher_node,
      # rviz_node,
      arm_node
    ]
  )