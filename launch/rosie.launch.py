from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
          default_value="arm/",
          description="Prefix for the HEBI Arm name. Usually the argument is not set",
      )
  )
  
  hebi_arm = LaunchConfiguration("hebi_arm")
  prefix = LaunchConfiguration("prefix")
  description_package = "hebi_description"

  # Include arm_node launch file
  arm_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution(
        [FindPackageShare('hebi_ros2_examples'), 'launch', 'arm_node.launch.py']
      )
    ),
    launch_arguments={
      'hebi_arm': hebi_arm,
      'prefix': LaunchConfiguration('prefix'),
      'params_file': 'rosie.yaml'
    }.items()
  )

  # Include omnibase_node launch file
  omnibase_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution(
        [FindPackageShare('hebi_ros2_examples'), 'launch', 'omni_base_node.launch.py']
      )
    ),
    launch_arguments={
      'params_file': 'rosie.yaml'
    }.items()
  )

  # joint_state_combiner.py node
  joint_state_combiner_node = Node(
    package='hebi_ros2_examples',
    executable='joint_state_combiner.py',
    name='joint_state_combiner_node',
    output='screen'
  )

  # Get URDF via xacro
  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          PathJoinSubstitution(
              [FindPackageShare(description_package), "urdf", "kits", "rosie.urdf.xacro"]
          ),
          " "
          "prefix:=",
          prefix,
          " ",
          "hebi_arm:=",
          hebi_arm,
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
    [FindPackageShare(description_package), "rviz", "hebi_arm.rviz"]
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
      arm_node,
      omnibase_node,
      joint_state_combiner_node,
      robot_state_publisher_node,
      rviz_node,
    ]
  )