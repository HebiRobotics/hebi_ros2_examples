from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.event_handlers import OnProcessExit
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
            default_value="true",
            description="Whether to start RViz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="hebi_description",
            description="Description package of the URDF. Usually the argument is not set, \
                        it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_package",
            default_value="hebi_description",
            description="Package of the arm's config file. Usually the argument is not set, \
                        it enables use of a custom config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value=PythonExpression(['"', LaunchConfiguration("hebi_arm"), '.cfg.yaml"']),
            description="Config file for the arm of type `.cfg.yaml`",
        )
    )

    hebi_arm = LaunchConfiguration("hebi_arm")
    prefix = LaunchConfiguration("prefix")
    description_package = LaunchConfiguration("description_package")
    config_package = LaunchConfiguration("config_package")
    config_file = LaunchConfiguration("config_file")
    use_rviz = LaunchConfiguration("use_rviz")

    # Generate URDF from config file
    urdf_output_dir = "/tmp/hebi"
    urdf_output_file_name = "hebi_arm.urdf.xacro"
    urdf_generator = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([FindPackageShare("hebi_description"), "scripts", "urdf_generator.py"]),
            PathJoinSubstitution([FindPackageShare(config_package), "config", "arms", config_file]),
            '--outputdir', urdf_output_dir,
            '--output-file-name', urdf_output_file_name
        ],
        output='screen'
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([urdf_output_dir, urdf_output_file_name]),
            " ",
            "prefix:=",
            prefix,
        ]
    )

    # Load the Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
        namespace=prefix,
    )

    # Define the arm node
    arm_node = Node(
        package="hebi_ros2_examples",
        executable="arm_node",
        name="arm_node",
        output="screen",
        parameters=[
            {"config_package": config_package},
            {"config_file": config_file},
            {"prefix": prefix},
        ],
        namespace=prefix,
    )

    # Load RViz
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "hebi_arm.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    launch_after_urdf_generator = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_generator,
            on_exit=[robot_state_publisher_node, arm_node, rviz_node],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            urdf_generator,
            launch_after_urdf_generator,
        ]
    )
