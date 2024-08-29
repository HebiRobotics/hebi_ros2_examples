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
            "config_file",
            default_value="None",  # Default set later
            description="Config file for the mobile_io of type `.cfg.yaml`",
        ),
        DeclareLaunchArgument(
            "config_package",
            default_value="None",  # Default set later
            description="Description package of the mobile_io_config file. Usually the argument is not set, \
                        it enables use of a custom description.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for the mobile_io. Usually the argument is not set",
        ),
    ]

    # Set default values for arguments
    default_config_package = 'hebi_description'
    default_config_file = 'mobile_io.cfg.yaml'
    default_arguments = [
        LogInfo(
            msg=f'Using default config package: {default_config_package}',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("config_package"), "None"))
        ),
        SetLaunchConfiguration(
            name="config_package",
            value=default_config_package,
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("config_package"), "None"))
        ),
        LogInfo(
            msg=f'Using default config file: {default_config_file}',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("config_file"), "None"))
        ),
        SetLaunchConfiguration(
            name="config_file",
            value=default_config_file,
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("config_file"), "None"))
        ),
    ]

    config_package = LaunchConfiguration("config_package")
    config_file = LaunchConfiguration("config_file")
    prefix = LaunchConfiguration("prefix")

    # Define the node
    mobile_io_node = Node(
        package='hebi_ros2_examples',
        executable='mobile_io_node.py',
        name='mobile_io_node',
        output='screen',
        parameters=[
            {'config_package': config_package},
            {'config_file': config_file}
        ],
        namespace=prefix,
    )

    return LaunchDescription(
        declared_arguments +
        default_arguments +
        [mobile_io_node]
    )
