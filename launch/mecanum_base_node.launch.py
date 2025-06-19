from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
  
  declared_arguments = []
  declared_arguments.append(
    DeclareLaunchArgument(
      "params_file",
      default_value="mecanum_base_params.yaml",
      description="Path to the YAML file containing the parameters for the base node.",
    )
  )

  params_file = LaunchConfiguration("params_file")

  robot_params = PathJoinSubstitution(
    [FindPackageShare('hebi_ros2_examples'), 'config', params_file]
  )

  mecanum_base_node = Node(
    package='hebi_ros2_examples',
    executable='mecanum_base_node',
    name='mecanum_base_node',
    output='screen',
    parameters=[
      robot_params
    ]
  )

  description_package = "hebi_description"

  # Get URDF via xacro
  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          PathJoinSubstitution(
              [FindPackageShare(description_package), "urdf", "kits", "mecanum_base.urdf.xacro"]
          )
      ]
  )

  robot_description = {"robot_description": robot_description_content}
  robot_state_publisher_node = Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      output="both",
      parameters=[robot_description],
  )

  rviz_config_file = PathJoinSubstitution(
    [FindPackageShare(description_package), "rviz", "hebi.rviz"]
  )
  rviz_node = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      output="log",
      arguments=["-d", rviz_config_file],
  )

  return LaunchDescription(
    declared_arguments +
    [
      mecanum_base_node,
      robot_state_publisher_node,
      rviz_node
    ]
  )