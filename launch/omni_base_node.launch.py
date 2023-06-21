import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  
  robot_params = PathJoinSubstitution(
    [FindPackageShare('hebi_ros2_examples'), 'config', 'omni_base_params.yaml']
  )

  omni_base_node = Node(
    package='hebi_ros2_examples',
    executable='omni_base_node',
    name='omni_base_node',
    output='screen',
    parameters=[
      robot_params
    ]
  )

  return LaunchDescription(
    [
      omni_base_node,
    ]
  )