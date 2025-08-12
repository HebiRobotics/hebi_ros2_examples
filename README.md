# HEBI ROS 2 Examples

HEBI Arms can be controlled using ROS 2 in three methods:
- Standalone HEBI API
- ROS 2 Control
- MoveIt

Each method offers unique advantages for different use cases and levels of integration with the ROS 2 ecosystem. The Standalone HEBI API provides direct control with HEBI C++ API, ROS 2 Control offers standardized interfaces, and MoveIt enables advanced motion planning capabilities.

For assistance or inquiries about implementing these control methods, please contact HEBI Robotics support at support@hebirobotics.com.

## Setting up your Workspace

Run the following commands to set up and download the HEBI ROS 2 packages:
```bash
# Create the workspace directory
mkdir -p ~/hebi_ws/src
cd ~/hebi_ws/src

# Install HEBI C++ ROS API package
# Option 1: Install the pre-built HEBI C++ API package
sudo apt-get install ros-$ROS_DISTRO-hebi-cpp-api
# Option 2: Clone the HEBI C++ API from source (if you prefer to build it yourself)
git clone https://github.com/HebiRobotics/hebi_cpp_api_ros.git

# Clone the HEBI description package (replace $ROS_DISTRO with 'humble', 'iron', or 'jazzy')
git clone -b ros2/$ROS_DISTRO https://github.com/HebiRobotics/hebi_description.git # ROS_DISTRO can be either humble, iron, or jazzy

# Clone the HEBI messages package
git clone https://github.com/HebiRobotics/hebi_msgs.git

# Clone this examples repository
git clone https://github.com/HebiRobotics/hebi_ros2_examples.git
```

Install the necessary dependencies using `rosdep`:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**NOTE:** If your ROS distribution is End-of-Life (EOL), you might need to include EOL distributions in your rosdep commands:
```bash
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -r -y --include-eol-distros
```

Finally, build the workspace and source it:
```bash
cd ~/hebi_ws
colcon build --symlink-install
source install/setup.bash
```

(Optional) Install `pip` dependencies for HRDF to URDF conversion script:

```bash
pip install -r src/hebi_ros2_examples/requirements.txt
```

You can skip this step in case you decide not to use the script. For more details, refer to [URDF Generation](#urdf-generation) section.

## Robot Description

For standalone control using the HEBI ROS 2 API, only an HRDF (HEBI Robot Description Format) file is required. See the [HEBI Documentation](https://docs.hebi.us/tools.html#robot-description-format) for a detailed explanation of the HRDF format.

However, for controlling using ROS 2 control, integrating with MoveIt, or simulating in environments such as Gazebo, a URDF file is necessary.

The `hebi_description` package provides both HRDFs and URDFs for the standard HEBI arm kits.

### Non-standard Kits

If you are using a non-standard HEBI kit:
1. **HRDF Creation:** Creating an HRDF file is relatively straightforward. Examine the standard HEBI kit HRDFs in the `config/arms/hrdf` directory of the `hebi_description` package for reference.
2. **URDF Generation:** A conversion script is available in the `hebi_description` repository to convert HRDF to URDF. Refer to the documentation provided in `hebi_description` for usage instructions.

### Important Note
While only HRDF is necessary when using the standalone ROS API, RViz visualization requires a URDF. The launch files include a `generate_urdf` argument (enabled by default) that automatically converts the HRDF to a URDF using the provided script, so you do not need to generate a URDF manually. See the [URDF Generation](#urdf-generation) section for more details.

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
  # IK Seed positions for inverse kinematics
  ik_seed_pos: [0.01, 1.0, 2.5, 1.5, -1.5, 0.01]

  # Home Position
  home_position: [0.0, 1.2, 1.8, 2.2, -1.57, 0.0]

  # Gripper specific settings
  has_gripper: true
  gripper_family: "customGripperFamily"
  gripper_name: "customGripperName"
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
- Ensure HRDF and gains file paths are relative to the config file
- `names` and `families` of your modules can be found and changed using [HEBI Scope](https://docs.hebi.us/tools.html#scope-gui)
- You can add `home_position` to `user_data` field for homing the arm on startup
- The gripper family/name need not be added to the `names` and `families`. By default, the gripper module will be identified using the first string in your families list and with the name "gripperSpool"
- For custom gripper family and name, you can specify them in the `user_data` field with keys `gripper_family` and `gripper_name` respectively

### Arm Node

The HEBI C++ API is wrapped in ROS 2 within the `arm_node` (`src/kits/arms/arm_node.cpp`). This node, utilizing the HEBI Arm API, provides various topics, services, and actions for arm control:

**Subscribers**
- */SE3_jog [hebi_msgs/msg/SE3Jog]*: Command end effector jog in SE3 space (cartesian and rotation)
- */cartesian_jog [hebi_msgs/msg/SE3Jog]*: Command end effector jog in cartesian space (x, y, z). Any angular displacement values set in the message are ignored.
- */cartesian_trajectory [trajectory_msgs/msg/JointTrajectory]*: Command a trajectory for the end effector in cartesian space
- */joint_jog [control_msgs/msg/JointJog]*: Command jog in joint angles
- */joint_trajectory [trajectory_msgs/msg/JointTrajectory]*: Command a trajectory in joint angles
- */cmd_ee_wrench [geometry_msgs/msg/Wrench]*: Command end effector wrench (force and torque) in the base frame
- */cmd_gripper [std_msgs/msg/Float64]*: Command gripper position (0 for fully open, 1 for fully closed)

**Publishers**
- */joint_states [sensor_msgs/msg/JointState]*: Joint angles of the arm and, if present, the gripper state (ranging from 0 for fully open to 1 for fully closed)
- */ee_pose [geometry_msgs/msg/PoseStamped]*: End effector pose in SE3 space
- */ee_wrench [geometry_msgs/msg/WrenchStamped]*: End effector wrench (force and torque) feedback in the base frame, calculated from torque errors
- */ee_force [geometry_msgs/msg/Vector3Stamped]*: End effector force (X, Y, Z components only), computed from the end effector position error. No scaling factor is applied; the output directly reflects the position errors.
- */gripper_state [std_msgs/msg/Float64]*: Gripper state (0 for fully open, 1 for fully closed)
- */inertia [geometry_msgs/msg/Inertia]*: Inertia of the arm
- */goal_progress [std_msgs/msg/Float64]*: Progress of the current goal of the arm (0.0 to 1.0)

**Action Servers**
- */arm_motion [hebi_msgs/action/ArmMotion]*: Command an arm trajectory in either joint space or SE3 space

**Services**
- */home [std_srvs/srv/Trigger]*: Home the arm
- */stop [std_srvs/srv/Trigger]*: Stop arm motion (cannot stop action execution; cancel the action instead)
- */gripper [std_srvs/srv/SetBool]*: Opens or closes the gripper (if available). Set to `true` to close the gripper and `false` to open it.

**Parameters**
- *config_package*: ROS package containing the config file
- *config_file*: Config file path relative to `config_package`
- *prefix*: Namespace for topics and prefix for joint names in `/joint_states`
- *use_gripper*: When true, enables gripper support. Ensure that the `has_gripper` parameter in the `user_data` section of your config file is also set to true.
- *compliant_mode*: When true, disables arm goals and sets joint efforts to zero for manual movement
- *ik_seed*: Sets the IK seed for inverse kinematic calculations
- *use_ik_seed*: When set to true, the node uses the IK seed specified by the `ik_seed` parameter for inverse kinematics calculations. If false, it uses the most recent joint feedback position as the IK seed.
- *use_traj_times*: When set to true, the node uses the trajectory times specified by the `traj_times` parameter for trajectory execution. If false, it uses a default time based on a heuristic.
- *topic_command_timeout*: Timeout in seconds for active topic commands. If no topic command is received within this time, the node resets the active command state, allowing new commands from actions or other topics.

**NOTE:** The `config_package`, `config_file`, `prefix`, and `use_gripper` parameters are set during launch and should not be changed during runtime. On the other hand, `compliant_mode`, `ik_seed`, `use_ik_seed`, `use_traj_times`, and `topic_command_timeout` are dynamic parameters.

### Launching the Arm Node

To launch the arm node, use:
```bash
ros2 launch hebi_ros2_examples arm.launch.py hebi_arm:=<your_robot_name>
```
**NOTE:** Do not forget to build your workspace and source your setup before running the above command.

#### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `hebi_arm` | (required) | Name of the robot to use |
| `config_package` | `hebi_description` | ROS package containing the config file |
| `config_file` | `<your_robot_name>.cfg.yaml` | Config file path relative to `config_package` |
| `prefix` | `""` | Namespace for topics and prefix for joint names |
| `use_gripper` | `false` | Whether to use the gripper (if available) |
| `use_rviz` | `true` | Whether to start RViz |
| `generate_urdf` | `true` | Generate URDF from HRDF or use pre-existing one |

#### URDF Generation

Both `arm.launch.py` and `arm_joystick_teleop.launch.py` include a parameter to control URDF generation:

- When `generate_urdf:=true` (default): The launch file will automatically generate a URDF from the HRDF file specified in your config, and save it in a cache directory (`~/.cache/hebi/hebi_arm.urdf.xacro`).
- When `generate_urdf:=false`: The launch file will use a pre-existing URDF from the description package located at `<description_package>/urdf/kits/<your_robot_name>.urdf.xacro`.

### Examples

To get you started, we have provided several example scripts that use the `arm_node`:

1. `move_arm.cpp`: A C++ example that publishes a predefined trajectory using the `arm_motion` action
2. `ex_publish_trajectory.py`: A Python example that publishes a predefined trajectory to the `/joint_trajectory` topic
3. `ex_teach_repeat_mobileio.py`: Uses HEBI Mobile IO to record and play trajectories, or go to saved waypoints
4. `ex_teleop_mobileio.py`: Uses HEBI Mobile IO to send jog commands to control the arm in real-time
5. `ex_haptic_teleop_node.py`: Uses a 3D Systems Touch X haptic device to control the arm in real-time with haptic feedback by sending jog commands while receiving force feedback from the `ee_wrench` topic

## ROS2 Control

### Additional Required Packages

To control HEBI arms using `ros2_control`, you need additional packages that aren't included in the basic setup:

```bash
# Clone required repositories
git clone https://github.com/HebiRobotics/hebi_hardware.git
git clone -b $ROS_DISTRO https://github.com/HebiRobotics/hebi_bringup.git # ROS_DISTRO can be humble, iron, or jazzy

# Install ROS2 Control dependencies
sudo apt install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers -y
```

### Required Configuration Files

For ROS 2 control integration, you'll need the following three types of files:

1. **ROS2 Control Macro File** - Defines hardware interfaces
2. **Combined URDF File** - Combines the macro with the existing URDF
3. **Controller Parameter File** - Configures controllers

For standard HEBI kits, these files are already provided in the `hebi_bringup` and `hebi_description` packages.

### ROS 2 Control Macro File

This file defines hardware interfaces and plugins for your robot. The template below shows the structure for a HEBI Arm ROS2 Control Macro file:
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
          <param name="gripper_joint_name">${prefix}<end_effector_name></param>    <!-- If you have a gripper -->
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
      <joint name="${prefix}<end_effector_name>">    <!-- If you have a gripper -->
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

According to the package conventions, this file should be named as `<your_robot_name>.ros2_control.xacro` and placed in `urdf/kits/ros2_control` folder of the `hebi_decription` package.

### ROS 2 Control URDF 

This file combines the ROS2 control macro with the main robot URDF. Here's a template:
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

According to the package conventions, this file should be named as `<your_robot_name>.urdf.xacro` and placed in `urdf/kits/ros2_control` folder of the `hebi_decription` package.

### ROS2 Control Parameter File

This YAML file configures the controllers used with your robot. Refer to the [ROS2 Controllers Documentation](https://control.ros.org/rolling/doc/ros2_controllers/doc/controllers_index.html#controllers-for-manipulators-and-other-robots) for detailed information.

Here's an example parameter file for the A-2580-06G arm:
```
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    hebi_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    
    gripper_controller:
      type: position_controllers/GripperActionController

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

gripper_controller:
  ros__parameters:
    joint: end_effector_1/input_l_finger
    state_publish_rate: 50.0 # Defaults to 50
    action_monitor_rate: 20.0 # Defaults to 20
```

If your arm does not include a gripper, you can omit the `gripper_controller` section from the configuration file.

### Launching HEBI Arm with ROS 2 Control

#### Hardware Execution

To launch the ROS 2 Control node with real hardware:

```bash
ros2 launch hebi_bringup bringup_arm.launch.py hebi_arm:=<your_robot_name> use_mock_hardware:=false use_gripper:=true/false
```

#### Simulated Execution

For testing without hardware (mock mode):

```bash
ros2 launch hebi_bringup bringup_arm.launch.py hebi_arm:=<your_robot_name>
```

#### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hebi_arm` | (required) | Name of the robot to use |
| `prefix` | `""` | Namespace for topics and prefix for joint names |
| `description_package` | `hebi_description` | Package containing the robot description files |
| `description_file` | `urdf/kits/ros2_control/<hebi_arm>.urdf.xacro` | Path to robot description file relative to `description_package` |
| `config_pkg` | `hebi_description` | Package containing the config file |
| `config_file_path` | `config/<hebi_arm>.cfg.yaml` | Path to config file relative to config_pkg |
| `controllers_package` | `hebi_bringup` | Package containing the controller parameter file |
| `controllers_file` | `config/<hebi_arm>_controllers.yaml` | Path to controller parameter file relative to `controllers_package` |
| `use_mock_hardware` | `true` | Use mock hardware interface instead of real hardware |
| `mock_sensor_commands` | `false` | Enable mock sensor commands (only applicable when `use_mock_hardware` is true) |
| `robot_controller` | `hebi_arm_controller` | Name of the robot controller to use |
| `use_gripper` | `false` | Whether to include a gripper controller in the setup |
| `use_rviz` | `true` | Launch RViz for visualization |

Here's an example to launch A-2580-06 arm with mock hardware:
```bash
ros2 launch hebi_bringup bringup_arm.launch.py hebi_arm:=A-2580-06
```

### Gazebo Classic Simulation

To launch your HEBI arm in Gazebo Classic simulation:

```bash
ros2 launch hebi_bringup bringup_arm_gazebo_classic.launch.py hebi_arm:=<your_robot_name>
```
**NOTE:** Do not forget to build your workspace and source your setup before running the above commands.

**Prerequisites:** Ensure you have Gazebo (`gazebo_ros`) and Gazebo ROS 2 Control (`gazebo_ros2_control`) installed. To install these packages, run:
```bash
sudo apt install ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-ros2-control
```

### Testing Controllers

After launching your arm in hardware or simulation, you can test the controllers:

```bash
ros2 launch hebi_bringup test_joint_trajectory_controller.launch.py config_file:=<test_config_file_path>
```

This launch file executes a trajectory controller test node, and uses the specified test configuration to define joint trajectories.

**Important Configuration:**
- The `config_file` parameter must reference a file in the `hebi_bringup/config` directory
- Default configuration (`test_goal_publishers_config.yaml`) is set for a 6-DoF arm
- For different arm configurations, edit the file to match your specific joint setup

When executed correctly, your robot arm will move through the joint positions defined in the config file.

### Gazebo (Ignition) Simulation

For the newer Gazebo (formerly Ignition) simulation:

```bash
ros2 launch hebi_bringup bringup_arm_gazebo.launch.py hebi_arm:=<your_robot_name>
```
**NOTE:** Do not forget to build your workspace and source your setup before running the above commands.

**Prerequisites:**
Ensure you have Gazebo (`ros_gz`) and Gazebo ROS 2 Control (`ign_ros2_control` / `gz_ros2_control`) installed. To install these packages, run:
- `sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control` for ROS 2 Humble
- `sudo apt install ros-$ROS_DISTRO-ros-gz ros-$ROS_DISTRO-gz-ros2-control` for ROS 2 Iron/Jazzy

To test the controller in Gazebo, use the same approach as with Gazebo Classic:

```bash
ros2 launch hebi_bringup test_joint_trajectory_controller.launch.py config_file:=<test_config_file_path>
```

## MoveIt

### Getting MoveIt Configurations

MoveIt requires additional configuration files (SRDF, controllers, kinematics, etc.) beyond what we've covered so far. For standard HEBI arm kits, these configurations are already available:

```bash
cd ~/hebi_ws/src
git clone https://github.com/HebiRobotics/hebi_moveit_configs.git
```

This repository contains ready-to-use MoveIt configurations for all standard HEBI arm kits. After cloning, do not forget to rebuild your workspace and source your setup.

### Custom Arm Configurations

For custom HEBI arm setups, you'll need to create your own MoveIt configuration package:

1. Use the MoveIt Setup Assistant to generate configuration files
2. Follow the detailed instructions in the [hebi_moveit_configs](https://github.com/HebiRobotics/hebi_moveit_configs) repository

The setup process involves defining planning groups, robot poses, and end-effectors for your specific arm configuration.

### Launching MoveIt with Hardware or Gazebo

The URDF files in the MoveIt config directory do not have access to HEBI Hardware plugin or Gazebo plugins defined during ROS2 control URDF setup. To simplify the process of modifying URDF, SRDF, and launch files, we provide `move_group.launch.py` in the `hebi_bringup` package.

We use this launch file in parallel with `bringup_arm.launch.py` to launch MoveIt.

#### Step 1: Launch Robot Control

Choose ONE of the following options:

**Option A: Real Hardware**
```bash
ros2 launch hebi_bringup bringup_arm.launch.py \
  hebi_arm:=<your_robot_name> \
  use_mock_hardware:=false \
  use_rviz:=false
```

**Option B: Gazebo Classic Simulation**
```bash
ros2 launch hebi_bringup bringup_arm_gazebo_classic.launch.py \
  hebi_arm:=<your_robot_name> \
  use_rviz:=false
```

**Option C: Gazebo (Ignition) Simulation**
```bash
ros2 launch hebi_bringup bringup_arm_gazebo.launch.py \
  hebi_arm:=<your_robot_name> \
  use_rviz:=false
```

#### Step 2: Launch MoveIt

After the robot control system is running, launch MoveIt:

```bash
ros2 launch hebi_bringup move_group.launch.py \
  hebi_arm:=<your_robot_name> \
  use_sim_time:=true/false
```

Set `use_sim_time:=true` when using simulation, and `use_sim_time:=false` with real hardware.

**Note:** We set `use_rviz:=false` in the first step to avoid duplicate RViz windows. The MoveIt launch file will open RViz with the MoveIt configuration loaded.

## Additional Resources

- [HEBI Documentation](https://docs.hebi.us) - Comprehensive documentation on HEBI modules and APIs
- [HEBI Forums](https://forum.hebi.us) - Community support and discussions
- [ROS 2 Control Documentation](https://control.ros.org) - Detailed information about ROS 2 control
- [MoveIt Documentation](https://moveit.ros.org) - Resources for using MoveIt

For further assistance, contact HEBI Robotics support at support@hebirobotics.com.
