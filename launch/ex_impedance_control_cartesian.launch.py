import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for the mobile_io and arm nodes. Usually the argument is not set",
        ),
    ]

    # Prefix and configuration parameters
    prefix = LaunchConfiguration("prefix")
    config_file = "ex_impedance_control_cartesian.cfg.yaml"
    config_package = "hebi_ros2_examples"

    # Include mobile_io.launch.py
    mobile_io_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(config_package), 'launch', 'mobile_io.launch.py']
        )),
        launch_arguments={
            'prefix': prefix,
            'config_file': config_file,
            'config_package': config_package,
        }.items()
    )

    # Include arm.launch.py
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(config_package), 'launch', 'arm.launch.py']
        )),
        launch_arguments={
            'prefix': prefix,
            'config_file': config_file,
            'config_package': config_package,
            # 'description_package': '',  # No description package
            'use_rviz': 'false',  # Disable RViz in the arm launch
        }.items()
    )

    # Load RViz with the specific config file from hebi_ros2_examples
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(config_package), "rviz", "ex_impedance_control_cartesian.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition("true"),  # Always launch RViz in this launch file
    )

    # Additional node: ex_impedance_control_cartesian_node
    cartesian_node = Node(
        package='hebi_ros2_examples',
        executable='ex_impedance_control_cartesian_node',
        name='ex_impedance_control_cartesian_node',
        namespace=prefix,
        output='screen',
        parameters=[
            {'config_package': config_package},
            {'config_file': config_file}
        ]
    )

    return LaunchDescription(
        declared_arguments +
        [
            mobile_io_launch,
            arm_launch,
            rviz_node,
            # cartesian_node,
        ]
    )
