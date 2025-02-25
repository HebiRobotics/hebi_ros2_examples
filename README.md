# HEBI ROS 2 Examples

HEBI Arms can be controlled using ROS 2 in three methods:
- Standalone HEBI API
- ROS 2 Control
- MoveIt

Each method offers unique advantages for different use cases and levels of integration with the ROS 2 ecosystem. The Standalone HEBI API provides direct control with HEBI C++ API, ROS 2 Control offers standardized interfaces, and MoveIt enables advanced motion planning capabilities.

For assistance or inquiries about implementing these control methods, please contact HEBI Robotics support at support@hebirobotics.com.

## Seting up your Workspace

Run the following commands to download the HEBI ROS 2 packages.
```
mkdir -p ~/hebi_ws/src
cd ~/hebi_ws/src
git clone https://github.com/HebiRobotics/hebi_cpp_api_ros.git
git clone -b ros2/$ROS_DISTRO https://github.com/HebiRobotics/hebi_description.git # ROS_DISTRO can be either humble, iron, or jazzy
git clone https://github.com/HebiRobotics/hebi_msgs.git
git clone https://github.com/HebiRobotics/hebi_ros2_examples.git
```

Install the necessary dependencies using `rosdep`:

```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

NOTE: If your ROS distribution is End-of-Life (EOL), you might need to include EOL distributions in your rosdep commands:
```
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -r -y --include-eol-distros
```

Finally, build the workspace and source it:
```
cd ~/hebi_ws
colcon build --symlink-install
source install/setup.bash
```

## Robot Description

For standalone control using the HEBI ROS 2 API, only an HRDF (HEBI Robot Description Format) file is required. See the [HEBI Documentation](https://docs.hebi.us/tools.html#robot-description-format) for a detailed explanation of the HRDF format.

However, for controlling using ROS 2 control, integrating with MoveIt, or simulating in environments such as Gazebo, a URDF file is necessary.

The `hebi_description` package provides both HRDFs and URDFs for the standard HEBI arm kits.

### Non-standard Kits

If you are using a non-standard HEBI kit:
1. **HRDF Creation:** Creating an HRDF file is relatively straightforward. Examine the standard HEBI kit HRDFs in the `config/arms/hrdf` directory of the `hebi_description` package for reference.
2. **URDF Generation:** A conversion script is available in the `hebi_description` repository to convert HRDF to URDF. Refer to the documentation provided in `hebi_description` for usage instructions.

### Important Note
While only HRDF is necessary when using the standalone ROS API, RViz visualization requires a URDF. However, the launch file automatically converts the HRDF into URDF using the provided script, eliminating the need for manual URDF generation.

## Standalone HEBI ROS 2 API

### HEBI Config File

Along with the robot description files, you need a HEBI configuration file (YAML) that specifies the parameters for connecting to the arm and defining its behavior. See the [HEBI Documentation](https://docs.hebi.us/tools.html#robot-config-format) for details on the config format.

The config files for standard HEBI kits are provided in the `hebi_description` package in the `config/arms` directory. 

For custom setups, you can create a new config file. Here's an example configuration for the A-2580-06G arm:
```
# T-Series 6-DoF Arm with Gripper
version: 1.0
families: ["Arm"]
names: ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]
hrdf: "hrdf/A-2580-06G.hrdf"

gains:
  default: "gains/A-2580-06.xml"
  gripper: "gains/A-2080-01.xml"

user_data:
  # Default seed positions for doing inverse kinematics
  ik_seed_pos: [0.01, 1.0, 2.5, 1.5, -1.5, 0.01]
  home_position: [0.0, 2.09, 2.09, 0.0, 1.57, 0.0]

  # Gripper specific settings
  has_gripper: true
  gripper_open_effort: 1
  gripper_close_effort: -5

plugins:
  - type: GravityCompensationEffort
    name: gravComp
    enabled: true
    ramp_time: 5

  - type: DynamicsCompensationEffort
    name: dynamicsComp
    enabled: true
    ramp_time: 5

  # Kits with a gas spring need to add a shoulder compensation torque.
  # It should be around -7 Nm for most kits, but it may need to be tuned
  # for your specific setup.
  - name: 'gasSpringCompensation'
    type: EffortOffset
    enabled: false
    ramp_time: 5
    offset: [0, -7, 0, 0, 0, 0]
```

Here are some key points to note:
- The HEBI configuration files follow this naming convention: `<your_robot_name>.cfg.yaml` and placed in the `config/arms` directory within the `hebi_description` package
- `names` and `families` of your modules can be found and changed using [HEBI Scope](https://docs.hebi.us/tools.html#scope-gui)
- Ensure HRDF and gains file paths are relative to the config file
- You can add `home_position` to `user_data` field for homing the arm on startup

### Arm Node

The HEBI C++ API is wrapped in ROS 2 within the `arm_node` (`src/kits/arms/arm_node.cpp`). This node, utilizing the HEBI Arm API, provides various topics, services, and actions for arm control:

**Subscribers**
- */SE3_jog [control_msgs/msg/JointJog]*: Command end effector jog in SE3 space (cartesian and rotation)
- */cartesian_jog [control_msgs/msg/JointJog]*: Command end effector jog in cartesian space (x, y, z)
- */cartesian_trajectory [trajectory_msgs/msg/JointTrajectory]*: Command a trajectory for the end effector in cartesian space
- */joint_jog [control_msgs/msg/JointJog]*: Command jog in joint angles
- */joint_trajectory [trajectory_msgs/msg/JointTrajectory]*: Command a trajectory in joint angles

**Publishers**
- */ee_pose [geometry_msgs/msg/PoseStamped]*: End effector pose in SE3 space
- */joint_states [sensor_msgs/msg/JointState]*: Joint angles of the arm
- */inertia [geometry_msgs/msg/Inertia]*: Inertia of the arm

**Action Servers**
- */arm_motion [hebi_msgs/action/ArmMotion]*: Command an arm trajectory in either joint space or SE3 space

**Services**
- */home [std_srvs/srv/Trigger]*: Home the arm
- */stop [std_srvs/srv/Trigger]*: Stop arm motion (cannot stop action execution; cancel the action instead)

**Parameters**
- *config_package*: ROS package containing the config file
- *config_file*: Config file path relative to `config_package`
- *prefix*: Namespace for topics and prefix for joint names in `/joint_states`
- *compliant_mode*: When true, disables arm goals and sets joint efforts to zero for manual movement
- *ik_seed*: Sets the IK seed for inverse kinematic calculations

**NOTE:** The `config_package`, `config_file` and `prefix` parameters are set during launch and should not be changed during runtime. On the other hand, `compliant_mode` and `ik_seed` are dynamic parameters.

### Launching the Arm Node

To launch the arm node, use:
```
ros2 launch hebi_ros2_examples arm.launch.py hebi_arm:=<your_robot_name>
```
**NOTE:** Do not forget to build your workspace and source your setup before running the above command.

By default, the `arm.launch.py` file sets `config_package` to `hebi_description` and `config_file` to `<your_robot_name>.cfg.yaml` (HEBI naming convention).

For custom config files, use the `config_package` and `config_file` parameters to specify your file location.

### Examples

To get you started, we have provided three example scripts that use the `arm_node`.

1. `move_arm`: This publishes a predefined trajectory using `arm_motion` action
2. `ex_publish_trajectory`: This publishes a predefined trajectory to the `/joint_trajectory` topic.
3. `ex_teach_repeat_mobileio`: This uses mobile IO to record and play trajetories, or go to saved waypoints.
4. `ex_teleop_mobileio`: This uses mobile IO to send jog commands to control the arm.

## ROS2 Control

To control HEBI arms using `ros2_control`, you need additional packages: `hebi_hardware` and `hebi_bringup`. Clone these repositories:
```
git clone https://github.com/HebiRobotics/hebi_hardware.git
git clone -b $ROS_DISTRO https://github.com/HebiRobotics/hebi_bringup.git # ROS_DISTRO can be humble, iron, or jazzy
```

Install dependencies:
```
sudo apt install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers -y
```

In addition to the main URDF file, you need three more files for ROS 2 control:
1. `ros2_control` macro file
2. URDF combining the macro file with existing URDF
3. Parameter file

### ROS 2 Control Macro File

The template for a HEBI Arm ROS2 Control Macro file is as follows:
```
<?xml version="1.0" encoding="UTF-8"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">

  <xacro:macro name="<your_robot_name>_ros2_control" params="
                name
                prefix
                use_mock_hardware:=^|false
                mock_sensor_commands:=^|false
                sim_gazebo_classic:=^|false     <!-- Not applicable in ROS 2 Jazzy -->
                sim_gazebo:=^|false
                families
                config_pkg
                config_file"
               >

    <ros2_control name="${name}" type="system">

      <hardware>
        <xacro:if value="${use_mock_hardware}">
          <plugin>mock_components/GenericSystem</plugin>
          <param name="mock_sensor_commands">${mock_sensor_commands}</param>
        </xacro:if>
        <xacro:if value="${sim_gazebo_classic}">    <!-- Not applicable in ROS 2 Jazzy -->
          <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </xacro:if>
        <xacro:if value="${sim_gazebo}">
          <plugin>ign_ros2_control/IgnitionSystem</plugin>    <!-- For ROS 2 Humble -->
          <plugin>gz_ros2_control/GazeboSimSystem</plugin>    <!-- For ROS 2 Iron/Jazzy -->
        </xacro:if>
        <xacro:unless value="${use_mock_hardware or sim_gazebo_classic or sim_gazebo}">     <!-- sim_gazebo_classic not applicable in ROS 2 Jazzy -->
          <param name="config_pkg">${config_pkg}</param>
          <param name="config_file">${config_file}</param>
          <plugin>hebi_hardware/HEBIHardwareInterface</plugin>
        </xacro:unless>
      </hardware>
      <joint name="joint_1">
        <command_interface name="position" />
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity" />
      </joint>
      <joint name="${prefix}<J1_name>">
        <command_interface name="position" />
        <command_interface name="velocity" />
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity">
          <param name="initial_value">0.0</param>
        </state_interface>
      </joint>
      ...
      ...
      ...
      <joint name="${prefix}<J1_name>">
        <command_interface name="position" />
        <command_interface name="velocity" />
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity">
          <param name="initial_value">0.0</param>
        </state_interface>
      </joint>
      <joint name="${prefix}<end_effector_name>"> <!-- If you have a gripper -->
        <command_interface name="position" />
        <command_interface name="velocity" />
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity">
          <param name="initial_value">0.0</param>
        </state_interface>
      </joint>

    </ros2_control>

    <!-- Gazebo Classic plugins -->
    ...

    <!-- Gazebo plugins -->
    ...

  </xacro:macro>
</robot>
```
**NOTE**: The gazebo classic and ignition plugins sections differ with each ROS version. Please refer to example files provided.

According to conventions, this file should be named as `<your_robot_name>.ros2_control.xacro` and placed in `urdf/kits/ros2_control` folder of the `hebi_decription` package.

### ROS 2 Control URDF 

The template for the URDF xacro file combining the `ros2_control` macro and the main URDF is as follows:
```
<?xml version="1.0" encoding="UTF-8"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="<your_robot_name>">

  <xacro:arg name="config_pkg" default="" />
  <xacro:arg name="config_file" default="" />
  <xacro:arg name="prefix" default="" />

  <xacro:arg name="use_mock_hardware" default="false" />
  <xacro:arg name="mock_sensor_commands" default="false" />
  <xacro:arg name="sim_gazebo_classic" default="false" />    <!-- Not applicable in ROS 2 Jazzy -->
  <xacro:arg name="sim_gazebo" default="false" />

  <xacro:include filename="/path/to/ros2_control_macro_file"/>

  <!-- create link fixed to the "world" -->
  <link name="world" />

  <joint name="world_to_base_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="world" />
    <child link="base_link" />
  </joint>

  <xacro:include filename="/path/to/original/URDF_file"/>
  
  <xacro:<your_robot_name>_ros2_control
    name="<your_robot_name>"
    use_mock_hardware="$(arg use_mock_hardware)"
    mock_sensor_commands="$(arg mock_sensor_commands)"
    sim_gazebo_classic="$(arg sim_gazebo_classic)"
    sim_gazebo="$(arg sim_gazebo)"
    config_pkg="$(arg config_pkg)"
    config_file="$(arg config_file)"
    prefix="$(arg prefix)" />

</robot>
```

According to conventions, this file should be named as `<your_robot_name>.urdf.xacro` and placed in `urdf/kits/ros2_control` folder of the `hebi_decription` package.

### ROS2 Control Parameter File

This file configures ROS 2 control parameters. Refer to the [ROS2 Control documentation](https://control.ros.org/rolling/doc/ros2_controllers/doc/controllers_index.html#controllers-for-manipulators-and-other-robots) for details.

Here's an example parameter file for the A-2580-06 arm:
```
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    hebi_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

hebi_arm_controller:
  ros__parameters:
    joints:
      - J1_base
      - J2_shoulder
      - J3_elbow
      - J4_wrist1
      - J5_wrist2
      - J6_wrist3
    
    command_interfaces:
      - position
      - velocity
    
    state_interfaces:
      - position
      - velocity

    state_publish_rate: 50.0 # Defaults to 50
    action_monitor_rate: 20.0 # Defaults to 20

    allow_partial_joints_goal: false # Defaults to false
    constraints:
      stopped_velocity_tolerance: 0.01 # Defaults to 0.01
      goal_time: 0.0 # Defaults to 0.0 (start immediately)
```

### Launching HEBI Arm with ROS 2 Control

To execute the ROS 2 Control node with the hardware, run the following command:
```
ros2 launch hebi_bringup bringup_arm.launch.py hebi_arm:=<your_robot_name> config_pkg:=<config_pkg> config_file_path:="<config_file_path>"
use_mock_hardware:=true/false
```
The default value of `use_mock_hardware` is true, and not setting `config_pkg` and `config_file_path` explicitly will default them to `hebi_description` and `config/<hebi_arm>.cfg.yaml` respectively.

Here's an example to launch A-2580-06 arm with mock hardware:
```
ros2 launch hebi_bringup bringup_arm.launch.py hebi_arm:=A-2580-06
```
There are other arguments that can be passed to the `bringup_arm.launch.py` file. Please refer the launch file to see all the parameters.

### Gazebo Classic

To execute the ROS 2 Control node with Gazebo classic simulation, run the following command:
```
ros2 launch hebi_bringup bringup_arm_gazebo_classic.launch.py hebi_arm:=<your_robot_name>
```
**NOTE:** Do not forget to build your workspace and source your setup before running the above commands. Also, ensure you have necessary packages installed such as `gazebo_ros`, `gazebo_ros2_control`

To test the controllers for your arm, use the following command:
```
ros2 launch hebi_bringup test_joint_trajectory_controller.launch.py config_file:=<test_config_file_path>
```

This launch file executes the publisher_joint_trajectory_controller from the ros2_controllers_test_nodes package, and uses the specified test configuration file to define the joint trajectories.

**NOTE:**
- The `config_file` parameter must point to a file within the `config` directory of the `hebi_bringup` package
- By default, `config_file` is set to `test_goal_publishers_config.yaml`, which is configured for a 6-DoF arm. Please edit this file to match your specific arm configuration before testing

Upon successful launch, you should observe the robotic arm moving according to the joint positions specified in the configuration file.

### Gazebo (Ignition)

To execute the ROS 2 Control node with Gazebo (ignition) simulation, run the following command:
```
ros2 launch hebi_bringup bringup_arm_gazebo.launch.py hebi_arm:=<your_robot_name>
```
**NOTE:** Do not forget to build your workspace and source your setup before running the above commands. Also, ensure you have necessary packages installed such as `ros_gz`, `ign_ros2_control` / `gz_ros2_control`.

To test the controller, you can utilize the same launch file as mentioned above (`test_joint_trajectory_controller.launch.py`).

## MoveIt

To use MoveIt with a HEBI Arm, you need SRDF and other configuration files required by MoveIt, along with the ROS 2 control repositories mentioned earlier. For standard HEBI kits, these files are provided and can be downloaded using the following command:
```
git clone https://github.com/HebiRobotics/hebi_moveit_configs.git
```

For custom setups, please refer to the [HebiRobotics/hebi_moveit_configs](https://github.com/HebiRobotics/hebi_moveit_configs) repository for detailed instructions on creating a MoveIt configuration package for your custom arm using the MoveIt Setup Assistant.

### MoveIt on Hardware / Gazebo

The URDF files in the MoveIt config directory do not have access to HEBI Hardware plugin or Gazebo plugins defined during ROS2 control URDF setup. To simplify the process of modifying URDF, SRDF, and launch files, we provide `move_group.launch.py` in the `hebi_bringup` package.

Use this launch file in parallel with `bringup_arm` launch files, either on hardware or Gazebo:

1. First, run one of the following commands:

```
ros2 launch hebi_bringup bringup_arm.launch.py hebi_arm:=<your_robot_name> use_mock_hardware:=false use_rviz:=false
```
or
```
ros2 launch hebi_bringup bringup_arm_gazebo_classic.launch.py hebi_arm:=<your_robot_name> use_rviz:=false
```
or
```
ros2 launch hebi_bringup bringup_arm_gazebo.launch.py hebi_arm:=<your_robot_name> use_rviz:=false
```

2. Then, launch the move_group:

```
ros2 launch hebi_bringup move_group.launch.py hebi_arm:=<your_robot_name> use_sim_time:=true/false
```

Set `use_sim_time` to `true` when using simulators like Gazebo.

We set `use_rviz:=false` in the first launch file to prevent duplicate RViz windows. After running the `move_group` launch command, you should see an RViz window with MoveIt loaded.
