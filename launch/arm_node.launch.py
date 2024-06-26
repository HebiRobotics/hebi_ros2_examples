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