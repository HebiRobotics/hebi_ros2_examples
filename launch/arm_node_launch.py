import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions


def generate_launch_description():
  arm_node = launch_ros.actions.Node(
    package='hebi_ros2_examples',
    executable='arm_node',
    name='arm_node',
    output='screen',
    parameters=[
      os.path.join(get_package_share_directory('hebi_ros2_examples'),
                  'config/A-2085-06_params.yaml'
                  )
    ]
  )

  return LaunchDescription([
    arm_node
  ])


if __name__ == '__main__':
  generate_launch_description()
