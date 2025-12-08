from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

  declared_arguments = []
  declared_arguments.append(
    DeclareLaunchArgument(
      "params_file",
      default_value="tready_base_params.yaml",
      description="Path to the YAML file containing the parameters for the tready base node.",
    )
  )

  params_file = LaunchConfiguration("params_file")

  robot_params = PathJoinSubstitution(
    [FindPackageShare('hebi_ros2_examples'), 'config', params_file]
  )

  treaded_base_node = Node(
    package='hebi_ros2_examples',
    executable='tready_node.py',
    name='tready_node',
    output='screen',
    parameters=[
      robot_params
    ]
  )

  return LaunchDescription(
    declared_arguments +
    [
      treaded_base_node
    ]
  )
