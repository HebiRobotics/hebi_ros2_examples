import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, LogInfo, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory  # Correct import


def setup_hebi_arm(context, *args, **kwargs):
    config_package = LaunchConfiguration('config_package').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    
    # Resolve the full path to the config file
    config_file_path = os.path.join(get_package_share_directory(config_package), 'config', config_file)

    # Load the YAML config file and extract the model name
    with open(config_file_path, 'r') as file:
        config_data = yaml.safe_load(file)
        hrdf_path = config_data['hrdf']
        hebi_arm = os.path.basename(hrdf_path).split('.')[0]  # Extract the model name (e.g., A-2085-06)
        print("yoooo")
        print(hebi_arm)

    return [SetLaunchConfiguration('hebi_arm', hebi_arm)]


def generate_launch_description():

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value="None",
            description="Config file for the mobile_io of type `.cfg.yaml`",
        ),
        DeclareLaunchArgument(
            "config_package",
            default_value="hebi_description",  # Default set to 'hebi_description'
            description="Description package of the mobile_io_config file. Usually the argument is not set, \
                        it enables use of a custom description.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="hebi_description",  # Default set to 'hebi_description'
            description="Description package of the mobile_io_config file. Usually the argument is not set, \
                        it enables use of a custom description.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for the mobile_io. Usually the argument is not set",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to start RViz.",
        )
    ]

    prefix = LaunchConfiguration("prefix")
    description_package = LaunchConfiguration("description_package")

    # Use OpaqueFunction to set up the hebi_arm variable dynamically
    setup_hebi_arm_action = OpaqueFunction(function=setup_hebi_arm)

    # Correct concatenation using PathJoinSubstitution and TextSubstitution
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "kits"]),
            "/",
            LaunchConfiguration('hebi_arm'),
            ".urdf.xacro ",
            " ",
            "prefix:=", # TODO: needs to be fixed/standardized
            prefix,
        ]
    )

    # Load the Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description_content}
        ],
        namespace=prefix,
        remappings=[('/joint_states', '/fdbk_joint_states')]
    )

    # Define the arm node
    arm_node = Node(
        package='hebi_ros2_examples',
        executable='arm_node',
        name='arm_node',
        output='screen',
        parameters=[
            {'config_package': LaunchConfiguration('config_package')},
            {'config_file': LaunchConfiguration('config_file')}
        ],
        namespace=prefix,
    )
    
    # Load RViz
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
        [
            setup_hebi_arm_action,  # Execute the setup action to set hebi_arm
            robot_state_publisher_node,
            arm_node,
            rviz_node
        ]
    )
