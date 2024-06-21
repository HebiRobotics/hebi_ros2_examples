import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals


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
          "use_rviz",
          default_value="false",
          description="Whether to start RViz.",
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
  declared_arguments.append(
      DeclareLaunchArgument(
          "params_file",
          default_value="None", # Default set later
          description="Path to the YAML file containing the parameters for the arm node.",
      )
  )

  # Set default values for arguments
  default_arguments = []
  default_arguments.append(
    LogInfo(
      msg=PythonExpression(['"Using default params_file: ', LaunchConfiguration("hebi_arm"), '_params.yaml"']),
      condition=LaunchConfigurationEquals("params_file", "None")
    )
  )
  default_arguments.append(
    SetLaunchConfiguration(
      name="params_file",
      value=PythonExpression(['"', LaunchConfiguration("hebi_arm"), '_params.yaml"']),
      condition=LaunchConfigurationEquals("params_file", "None")
    )
  )
  
  hebi_arm = LaunchConfiguration("hebi_arm")
  description_package = LaunchConfiguration("description_package")
  prefix = LaunchConfiguration("prefix")
  params_file = LaunchConfiguration("params_file")
  use_rviz = LaunchConfiguration("use_rviz")
  
  robot_params = PathJoinSubstitution(
    [FindPackageShare('hebi_ros2_examples'), 'config', params_file]
  )

  rviz_config_file = PathJoinSubstitution(
    [FindPackageShare("hebi_ros2_examples"), "config/rviz", "magnet_arm.rviz"]
  )
  rviz_node = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      output="log",
      arguments=["-d", rviz_config_file],
      condition=IfCondition(use_rviz),
  )

  effortviz_arm = Node(
    package='hebi_ros2_examples',
    executable='effortviz_arm',
    name='effortviz_arm',
    output='screen',
    parameters=[
      robot_params,
      {"prefix": prefix}
    ],
    namespace=prefix,
  )

  # Other launch files must be included at the end to avoid argument reinitialization
  arm_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('hebi_ros2_examples'),
                'launch',
                'arm_node.launch.py'
            ])
        ]),
        launch_arguments={
            "hebi_arm": hebi_arm,
            "prefix": prefix,
            "use_rviz": "false",
            "description_package": description_package,
            "params_file": params_file,
        }.items(),
    )

  return LaunchDescription(
    declared_arguments +
    default_arguments +
    [
      rviz_node,
      effortviz_arm,
      arm_node_launch
    ]
  )