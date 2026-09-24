import os
from launch import LaunchDescription
from launch.actions import SetLaunchConfiguration, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, EqualsSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "names",
            description="List of names for modules in the group"
        ),
        DeclareLaunchArgument(
            "families",
            default_value="['HEBI']",
            description="List of families for modules in the group"
        ),
        DeclareLaunchArgument(
            "gains_package",
            default_value="",
            description="ROS2 package with the gains file to set at startup",
        ),
        DeclareLaunchArgument(
            "gains_file",
            default_value="",
            description="Package-relative file path to the gains file to set at startup",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for the node. Usually the argument is not set",
        ),
    ]

    names = LaunchConfiguration("names")
    families = LaunchConfiguration("families")
    prefix = LaunchConfiguration("prefix")

    # Define the node
    group_node = Node(
        package='hebi_ros2_examples',
        executable='group_node.py',
        name='group_node',
        output='screen',
        parameters=[
            {'names': names},
            {'families': families}
        ],
        namespace=prefix,
    )

    return LaunchDescription(
        declared_arguments +
        [group_node]
    )
