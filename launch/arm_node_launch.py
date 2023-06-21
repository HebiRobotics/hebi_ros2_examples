import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  declared_arguments = []
  declared_arguments.append(
    DeclareLaunchArgument(
      "robot_name",
      description="Name of the robot to be used.",
    )
  )

  declared_arguments.append(
      DeclareLaunchArgument(
          "description_package",
          default_value="hebi_description",
          description="Description package with robot URDF/xacro files. Usually the argument \
      is not set, it enables use of a custom description.",
      )
  )
  declared_arguments.append(
      DeclareLaunchArgument(
          "description_file",
          default_value="A-2085-06.urdf.xacro",
          description="URDF/XACRO description file with the robot.",
      )
  )

  description_package = LaunchConfiguration("description_package")
  description_file = LaunchConfiguration("description_file")
  
  robot_name = LaunchConfiguration("robot_name")

  robot_params = PathJoinSubstitution(
    [FindPackageShare('hebi_ros2_examples'), 'config', PythonExpression(['"', robot_name, '_params.yaml"'])]
  )

  # Get URDF via xacro
  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          PathJoinSubstitution(
              [FindPackageShare(description_package), "urdf", "kits", description_file]
          ),
          " ",
          "config_pkg:=",
          description_package,
          " ",
      ]
  )

  robot_description = {"robot_description": robot_description_content}

  rviz_config_file = PathJoinSubstitution(
      [FindPackageShare(description_package), "rviz", "A-2085-06.rviz"]
  )

  robot_state_pub_node = Node(
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
      robot_state_pub_node,
      rviz_node,
      arm_node
    ]
  )