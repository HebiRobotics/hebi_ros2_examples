import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, OpaqueFunction, LogInfo
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


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
            "use_gripper",
            default_value="false",
            description="Whether to use the gripper."
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
            default_value="false",
            description="Generate URDF from HRDF or use pre-existing one."
        ),
    ]

    # Configuration variables
    hebi_arm = LaunchConfiguration("hebi_arm")
    prefix = LaunchConfiguration("prefix")
    description_package = LaunchConfiguration("description_package")
    config_package = LaunchConfiguration("config_package")
    config_file = LaunchConfiguration("config_file")
    use_gripper = LaunchConfiguration("use_gripper")
    use_rviz = LaunchConfiguration("use_rviz")
    generate_urdf = LaunchConfiguration("generate_urdf")

    # URDF paths
    generated_urdf_path = PathJoinSubstitution([
        TextSubstitution(text=os.path.expanduser('~')), 
        '.cache', 
        'hebi', 
        'hebi_arm.urdf.xacro'
    ])
    preexisting_urdf_path = PathJoinSubstitution([
        FindPackageShare(description_package), 
        'urdf', 
        'kits',
        PythonExpression(['"', hebi_arm, '.urdf.xacro"'])
    ])

    # Conditional URDF path selection
    urdf_path = PythonExpression([
        '"', generated_urdf_path, '" if "', generate_urdf, 
        '" == "true" else "', preexisting_urdf_path, '"'
    ])

    # URDF generation (if enabled)
    urdf_generator = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([FindPackageShare("hebi_description"), "scripts", "urdf_generator.py"]),
            PathJoinSubstitution([FindPackageShare(config_package), "config", "arms", config_file]),
            '--outputdir', PathJoinSubstitution([os.path.expanduser('~'), '.cache', 'hebi']),
            '--output-file-name', 'hebi_arm.urdf.xacro'
        ],
        output='screen',
        condition=IfCondition(generate_urdf),
        on_exit=[LogInfo(msg="URDF generation completed successfully")]
    )

    # Robot description command
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        urdf_path,
        " ",
        "prefix:=",
        prefix,
    ])

    # Node definitions
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
        namespace=prefix,
    )

    # Arm Node
    arm_node = Node(
        package="hebi_ros2_examples",
        executable="arm_node",
        name="arm_node",
        output="screen",
        parameters=[
            {"config_package": config_package},
            {"config_file": config_file},
            {"prefix": prefix},
            {"use_gripper": use_gripper},
        ],
        namespace=prefix,
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d", PathJoinSubstitution([
                FindPackageShare(description_package), 
                "rviz", 
                "hebi.rviz"
            ])
        ],
        condition=IfCondition(use_rviz),
    )

    # Event handler for URDF generation path
    launch_after_urdf_generator = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_generator,
            on_exit=[robot_state_publisher_node, arm_node, rviz_node]
        )
    )

    # Opaque function to handle conditional logic
    def launch_conditional_nodes(context, *args, **kwargs):
        nodes_to_launch = []
        
        if context.launch_configurations['generate_urdf'] == 'true':
            # Log before adding event handler
            nodes_to_launch.append(LogInfo(msg="Generating new URDF from configuration..."))
            nodes_to_launch.append(launch_after_urdf_generator)
        else:
            # Log before adding nodes
            nodes_to_launch.append(LogInfo(
                msg="Using pre-existing URDF: " + 
                context.launch_configurations['description_package'] +
                "/urdf/kits/" +
                context.launch_configurations['hebi_arm'] + 
                ".urdf.xacro"
            ))
            nodes_to_launch.extend([
                robot_state_publisher_node,
                arm_node,
                rviz_node
            ])
        return nodes_to_launch

    # Build launch description
    ld = LaunchDescription(declared_arguments)
    
    # Always add URDF generator (conditionally runs)
    ld.add_action(urdf_generator)
    
    # Add conditional logging and node launching
    ld.add_action(OpaqueFunction(function=launch_conditional_nodes))

    return ld
