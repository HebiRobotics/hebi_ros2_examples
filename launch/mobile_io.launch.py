import os
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

# Import utility functions
from hebi_ros2_examples.config_utils import convert_to_ros_params

def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    config_package = LaunchConfiguration("config_package").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)

    # Resolve the full path to the config file
    mobile_io_config = PathJoinSubstitution(
        [FindPackageShare(config_package), 'config', config_file]
    )

    # Convert the config file on the fly
    converted_config_file = convert_to_ros_params(context, mobile_io_config, 'mobile_io_node')

    mobile_io_node = Node(
        package='hebi_ros2_examples',
        executable='mobile_io_node.py',
        name='mobile_io_node',
        output='screen',
        parameters=[
            converted_config_file,  # This includes all parameters from the converted YAML file
            {'config_package': LaunchConfiguration('config_package')}  # Add the config_package parameter
        ],
        namespace=prefix,
    )

    return [mobile_io_node]


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
    default_arguments = [
        LogInfo(
            msg=PythonExpression(['"Using default config package: ', default_config_package, '"']),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("config_package"), "None"))
        ),
        SetLaunchConfiguration(
            name="config_package",
            value=PythonExpression(['"', default_config_package, '"']),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("config_package"), "None"))
        ),
        LogInfo(
            msg=PythonExpression(['"Using default config file: ', 'mobile_io.cfg.yaml', '"']),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("config_file"), "None"))
        ),
        SetLaunchConfiguration(
            name="config_file",
            value=PythonExpression(['"', 'mobile_io.cfg.yaml', '"']),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration("config_file"), "None"))
        ),
    ]

    return LaunchDescription(
        declared_arguments +
        default_arguments +
        [OpaqueFunction(function=launch_setup)]
    )
