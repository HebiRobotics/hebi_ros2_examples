from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "hebi_arm",
            description="Name of the robot to be used."
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for the HEBI Arm name."
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to start RViz."
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="hebi_description",
            description="Description package of the URDF. Usually the argument is not set, \
                         it enables use of a custom description."
        ),
        DeclareLaunchArgument(
            "config_package",
            default_value="hebi_description",
            description="Configuration package."
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PythonExpression(['"', LaunchConfiguration("hebi_arm"), '.cfg.yaml"']),
            description="Config file for the arm of type `.cfg.yaml`."
        ),
        DeclareLaunchArgument(
            "generate_urdf",
            default_value="true",
            description="Generate URDF from HRDF or use pre-existing one."
        ),
    ]

    # Configuration variables
    hebi_arm = LaunchConfiguration("hebi_arm")
    prefix = LaunchConfiguration("prefix")
    description_package = LaunchConfiguration("description_package")
    config_package = LaunchConfiguration("config_package")
    config_file = LaunchConfiguration("config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    generate_urdf = LaunchConfiguration("generate_urdf")

    joystick_node = Node(
        package="hebi_ros2_examples",
        executable="joystick_node",
        name="joystick_node",
        output="screen",
        parameters=[{"prefix": prefix}],
        namespace=prefix,
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen"
    )

    # Other launch files must be included at the end to avoid argument reinitialization
    arm_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("hebi_ros2_examples"), "launch", "arm.launch.py"])]
        ),
        launch_arguments={
            "hebi_arm": hebi_arm,
            "prefix": prefix,
            "use_rviz": use_rviz,
            "description_package": description_package,
            "config_package": config_package,
            "config_file": config_file,
            "generate_urdf": generate_urdf,
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [
            joystick_node,
            joy_node,
            arm_node,
        ]
    )
