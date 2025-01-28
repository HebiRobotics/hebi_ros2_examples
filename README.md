# HEBI ROS 2 Examples

We provide three ways to control HEBI Arms using ROS 2:
- Standalone HEBI API
- ROS 2 Control
- MoveIt

## Standalone HEBI ROS 2 API

Set up your workspace using the following commands:
```
mkdir -p ~/hebi_ws/src
cd ~/hebi_ws/src
git clone -b ros2 https://github.com/HebiRobotics/hebi_cpp_api_ros.git
git clone -b ros2/$ROS_DISTRO https://github.com/HebiRobotics/hebi_description.git # ROS_DISTRO can be either humble, iron, or jazzy
git clone https://github.com/HebiRobotics/hebi_msgs.git
git clone https://github.com/HebiRobotics/hebi_ros2_examples.git
```

Once you have cloned the necessary repositories, install the required dependencies using rosdep
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

NOTE: If your ROS distribution is End-of-Life (EOL), you might need to include EOL distributions in your rosdep commands:
```
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -r -y --include-eol-distros
```

Then, build the workspace and source it using the following commands.
```
cd ~/hebi_ws
colcon build --symlink-install
source install/setup.bash
```

### Create HRDF

To control your HEBI arm using the standalone API, you do not require a URDF file for robot description. Instead, we use a HRDF to describe the HEBI Arm. For more information, refer https://docs.hebi.us/tools.html#robot-description-format.

For example, the HRDF for A-2085-05 arm is shown below.
```
<?xml version="1.0" encoding="UTF-8"?>
<!-- For documentation on the HEBI Robot Configuration please visit: -->
<!-- https://github.com/HebiRobotics/hebi-hrdf/blob/master/FORMAT.md -->

<!-- X-Series Arm (5-DoF) -->
<robot version="1.2.0">
  <actuator type="X8-9"/>
  <bracket type="X5HeavyRightOutside"/>
  <actuator type="X8-16"/>
  <link type="X5" extension="0.325" twist="pi"/>
  <actuator type="X8-9"/>
  <link type="X5" extension="0.325" twist="pi"/>
  <actuator type="X5-1"/>
  <bracket type="X5LightRight"/>
  <actuator type="X5-1"/>
  
  <!-- For custom end-effector parameters and formatting please visit: -->
  <!-- https://github.com/HebiRobotics/hebi-hrdf/blob/master/FORMAT.md#end-effector -->
  <end-effector />
  
</robot>
```

### Arm Node

To control the arm with our `arm_node`, you need to create a HEBI config file, which is a YAML file. It contains all the parameters required to connect with the arm and also define its behavior (refer https://docs.hebi.us/tools.html#robot-config-format). An example config for the A-2085-05 arm is given below:
```
# X-Series 5-DoF Arm
version: 1.0
families: ["Arm"]
names: ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2"]
hrdf: "hrdf/A-2085-05.hrdf"

gains:
  default: "gains/A-2085-05.xml"

user_data:
  # Default seed positions for doing inverse kinematics
  ik_seed_pos: [0.01, 1.0, 2.5, 1.5, -1.5]
  home_position: [0.0, 2.09, 2.09, 1.57, 0.0]

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
    offset: [0, -7, 0, 0, 0]

```
**NOTE:** `names` and `families` of your modules can be found and changed using Scope.

The `arm_node` takes in the following parameters:
```
arm_node:
  ros__parameters:
    prefix:
    config_package:
    config_file:
```

The parameters are defined as follows -
- `config_package`: ROS package where the config file is stored
- `config_file`: Relative path of the config file from the config_package
- `prefix`: Namespace for the topics and prefix for the joint names that will be published in the /joint_state topic

By default, the `arm.launch.py` launch file sets the `config package` parameter to the `hebi_description` package and the config file to `<your_robot_name>.cfg.yaml` file. Hence, it is recommended to name using these conventions or set the parameters while launching.

You can launch the arm node using the following command:
```
ros2 launch hebi_ros2_examples arm.launch.py hebi_arm:=<your_robot_name>
```

**NOTE:** Do not forget to build your workspace and source your setup before running the above command.

The arm node uses HEBI Arm API. It provides a variety of topics, services, and action for the user to control the arm.

**Subscribers**
- */SE3_jog [control_msgs/msg/JointJog]*: To command end effector jog in SE3 space (cartesian and rotation)
- */cartesian_jog [control_msgs/msg/JointJog]*: To command end effector jog in cartesian space (x, y, z)
- */cartesian_trajectory [trajectory_msgs/msg/JointTrajectory]*: To command a trajectory for the end effector in cartesian space
- */joint_jog [control_msgs/msg/JointJog]*: To command jog in joint angles
- */joint_trajectory [trajectory_msgs/msg/JointTrajectory]*: To command a trajectory in joint angles

**Publishers**
- */ee_pose [geometry_msgs/msg/PoseStamped]*: The end effector pose in SE3 space
- */joint_states [sensor_msgs/msg/JointState]*: Joint angles of the arm
- */inertia [geometry_msgs/msg/Inertia]*: Inertia of the arm

**Action Servers**
- */arm_motion [hebi_msgs/action/ArmMotion]*

**Services**
- */home [std_srvs/srv/Trigger]*: Service to home the arm
- */stop [std_srvs/srv/Trigger]*: Service to stop arm motion. This service cannot stop an action execution. Actions can be stopped only by canceling the action.

Apart from the parameters set using the file discussed above, the arm node also uses few extra parameters which you can set dynamically:
- *compliant_mode*: Setting it to true disables any goal set to the arm and sets the joint efforts to zero for easy manual movement of the arm.
- *ik_seed*: This parameter sets the IK seed for inverse kinematic calculations.

To get you started, we have provided three example scripts that use the `arm_node`.

1. `move_arm`: This publishes a predefined trajectory using `arm_motion` action
2. `ex_publish_trajectory`: This publishes a predefined trajectory to the `/joint_trajectory` topic.
3. `ex_teach_repeat_mobileio`: This uses mobile IO to record and play trajetories, or go to saved waypoints.
4. `ex_teleop_mobileio`: This uses mobile IO to send jog commands to control the arm.

## HRDF to URDF

**NOTE: Still in development. the URDFs for the standard arm kits are  already provided. Contact support@hebirobotics.com if you need help with generating URDF**

If you require ROS 2 control or MoveIt functionalities, you will need to have a URDF of the HEBI Arm. To help you with that, we have a script that converts HRDF to URDF, provided in `hebi_description` package.

The URDF for A-2085-05 looks like this
```
<?xml version='1.0' encoding='UTF-8'?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="A-2085-05">
  
  <!-- HEBI A-2085-05 Arm Kit -->
  <xacro:include filename="$(find hebi_description)/urdf/components/hebi.xacro"/>

  <link name="base_link" />

  <joint name="base_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link" />
    <child link="J1_base/INPUT_INTERFACE"/>
  </joint>

  <xacro:actuator type="X8_9" name="J1_base" child="shoulder_bracket"/>
  <xacro:bracket type="X5HeavyRightOutside" name="shoulder_bracket" child="J2_shoulder"/>
  <xacro:actuator type="X8_16" name="J2_shoulder" child="shoulder_elbow"/>
  <xacro:link type="X5" extension="0.325" twist="${pi}" name="shoulder_elbow" child="J3_elbow"/>
  <xacro:actuator type="X8_9" name="J3_elbow" child="elbow_wrist1"/>
  <xacro:link type="X5" extension="0.325" twist="${pi}" name="elbow_wrist1" child="J4_wrist1"/>
  <xacro:actuator type="X5_1" name="J4_wrist1" child="wrist2_bracket"/>
  <xacro:bracket type="X5LightRight" name="wrist2_bracket" child="J5_wrist2"/>
  <xacro:actuator type="X5_1" name="J5_wrist2" child="end_effector"/>
  <xacro:gripper type="Custom" name="end_effector" mass="0.005"/>

</robot>
```

To be compatible with the `hebi_description` package standards, the URDF will be saved as `<robot_name>.urdf.xacro` in the `urdf/kits` folder.

You can visualize the arm URDF using RViz. To do so, run the following command.
```
ros2 launch hebi_description view_arm.launch.py hebi_arm:=<your_robot_name>
```

**NOTE:** Do not forget to build your workspace and source your setup before running the above command.

If the launch is successful, you should see a RViz window, with the arm and a GUI to control the joints of the arm.

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/hebi_description_1.png)

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/hebi_description_2.png)

## ROS2 Control

To control HEBI arm using `ros2_control`, you need additional packages.
```
git clone https://github.com/HebiRobotics/hebi_hardware.git
git clone -b $ROS_DISTRO https://github.com/HebiRobotics/hebi_bringup.git # ROS_DISTRO can be either humble, iron, or jazzy
```

Install dependencies:
```
sudo apt install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers
```

You will also need three more files apart from the URDF file describing the arm:
- ROS2 Control macro file
- Xacro file comprising the ROS 2 Control macro file and the original URDF file
- ROS2 Control parameter file

Both of these files should be placed in `urdf/kits/ros2_control` folder of the `hebi_decription` package.

### ROS2 Control Macro File

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
        <xacro:unless value="${use_mock_hardware or sim_gazebo_classic or sim_gazebo}">
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
      ...
      ...
      ...
      <joint name="joint_n">
        <command_interface name="position" />
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity" />
      </joint>
    </ros2_control>

    <!-- Gazebo Classic plugins -->
    ...

    <!-- Gazebo plugins -->
    ...

  </xacro:macro>
</robot>
```
Name this file `<your_robot_name>_macro.ros2_control.xacro`.

**NOTE**: The gazebo classic and ignition plugins sections differ with each ROS version. Please refer to example files provided.

### Combined URDF Xacro File

The template for the combined URDF xacro file is given below:
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
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="world" />
    <child link="base_link"/>
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

This file is named similarly to the original URDF file, i.e., `<robot_name>.urdf.xacro`, but saved in the `urdf/kits/ros2_control` folder.

### ROS2 Control Parameter File

More details about the ROS2 Control Parameter file can be found in the [ROS2 Control documentation](https://ros-controls.github.io/control.ros.org/getting_started.html).

The parameter file for A-2085-05 looks like this:
```
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

    hebi_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

forward_position_controller:
  ros__parameters:
    joints:
      - J1_base
      - J2_shoulder
      - J3_elbow
      - J4_wrist1
      - J5_wrist2
    interface_name: position

hebi_arm_controller:
  ros__parameters:
    joints:
      - J1_base
      - J2_shoulder
      - J3_elbow
      - J4_wrist1
      - J5_wrist2
    
    command_interfaces:
      - position
    
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

To execute the ROS 2 Control node with the hardware, run the following command:
```
ros2 launch hebi_bringup bringup_arm.launch.py hebi_arm:=<your_robot_name> config_pkg:=<config_pkg> config_file_path:="<config_file_path>"
use_mock_hardware:=true/false
```
The default value of `use_mock_hardware` is true, and not setting `config_pkg` and `config_file_path` explicitly will default them to `hebi_description` and `config/<hebi_arm>.cfg.yaml` respectively.

Here is an example to launch A-2085-05 arm with mock hardware:
```
ros2 launch hebi_bringup bringup_arm.launch.py hebi_arm:=A-2085-05
```
There are other arguments that can be passed to the `bringup_arm.launch.py` file. Please refer the launch file to see all the parameters.

### Gazebo Classic

To execute the ROS 2 Control node with Gazebo classic simulation, run the following command:
```
ros2 launch hebi_bringup bringup_arm_gazebo_classic.launch.py hebi_arm:=<your_robot_name>
```
**NOTE:** Do not forget to build your workspace and source your setup before running the above commands. Also, ensure you have necessary packages installed such as `gazebo_ros`, `gazebo_ros2_control`

To test the controllers, you can do so by running the following command:
```
ros2 launch hebi_bringup test_joint_trajectory_controller.launch.py
```
You should see the arm moving according to the joint positions given in the configuration file.

### Gazebo (Ignition)

To execute the ROS 2 Control node with Gazebo (ignition) simulation, run the following command:
```
ros2 launch hebi_bringup bringup_arm_gazebo.launch.py hebi_arm:=<your_robot_name>
```
**NOTE:** Do not forget to build your workspace and source your setup before running the above commands. Also, ensure you have necessary packages installed such as `ros_gz`, `ign_ros2_control` / `gz_ros2_control`.

You can use the same launch file `test_joint_trajectory_controller.launch.py` to test the controller.

## MoveIt

To use MoveIt with an HEBI Arm, you will need SRDF and other configuration files that MoveIt requires along with the ROS 2 control repositories mentioned above. These files for the standard HEBI kits are provided and can be downloaded using the following command:
```
git clone -b ros2 https://github.com/HebiRobotics/hebi_moveit_configs.git
```

However, if you have a custom setup, you will need to create a MoveIt configuration package for your arm using the MoveIt Setup Assistant.

### Create MoveIt Configuration Package

Open MoveIt Setup Assistant using the following command:
```
ros2 run moveit_setup_assistant moveit_setup_assistant 
```

The following window will open:

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/moveit_setup_assistant_1.png)

Click on the `Create New MoveIt Configuration Package` button and load the URDF xacro file created in the first step, from the folder `urdf/kits` of `hebi_description` package. Finally, click on the `Load Files` button.

Once the files are loaded, you will see the following window:

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/moveit_setup_assistant_2.png)

Click on `Self-Collisions` on the sidebar, and click on `Generate Collision Matrix` button after choosing appropriate Sampling Density. This will generate the collision matrix for your arm.

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/moveit_setup_assistant_3.png)

Click on `Virtual Joints` on the sidebar, and click on `Add Virtual Joint` button to add a virtual joint from world to the base_link of your arm. Name the virtual joint as `world_joint`. Choose the `Child Link` as `base_link` and type `world` in the `Parent Frame Name`. Keep the `Joint Type` as fixed, and click on `Save` button. Finally, you will see the following window:

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/moveit_setup_assistant_4.png)

Click on `Planning Groups` on the sidebar, and click on `Add Group` button to add a planning group for your arm. Name the planning group as `hebi_arm`. Choose the kinematics solver as desired. Leave the remaining fields as default. 

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/moveit_setup_assistant_5.png)

Now, click on `Add joints` button to add the joints of your arm to the planning group. Select all of the actuator joints and click on the right arrow button to add them to the Planning Group. Finally, click "Save" and you will see the following window:

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/moveit_setup_assistant_6.png)

Click on `Robot Poses` on the sidebar, and click on `Add Pose` button to add a robot pose for your arm. Name the robot pose as `home`, and set the joint angles for that position. Finally, click "Save".

It is recommended to add another robot pose to test MoveIt later. We add one more robot pose `test`, and the final window looks like this:

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/moveit_setup_assistant_7.png)

Click on `ROS 2 Controllers` on the sidebar and click on `Auto Add JointTrajectoryController Controllers For Each Planning Group` button. This will add the necessary ROS 2 controllers for your arm.

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/moveit_setup_assistant_8.png)

Similarly, click on `MoveIt Controllers` on the sidebar and click on `Auto Add FollowJointsTrajectory Controllers For Each Planning Group` button. This will add the necessary MoveIt controllers for your arm.

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/moveit_setup_assistant_9.png)

Go to the `Author Information` tab and fill in the necessary details.

Click on the `Configuration Files` tab, and select the desired output directory. Note that the files will be placed directly in the selected directory, not in a new subdirectory, so choose a directory matching convention such as `<your_robot_name>_moveit_config`. Click on the `Generate Package` button to generate the MoveIt configuration package for your arm. Once the package is generated, you will see the following window:

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/moveit_setup_assistant_10.png)

Finally, click on `Exit Setup Assistant` button to exit the MoveIt Setup Assistant.

To test the MoveIt configuration, run the following command:
```
ros2 launch <your_robot_name>_moveit_config demo.launch.py
```
**NOTE:** Do not forget to build your workspace and source your setup before running the above command.

If the launch is successful, a RViz window opens up with the arm and a GUI to control the arm that looks like this:

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/moveit_1.png)

To check if the MoveIt is able to plan and execute the trajectories, click on the `Planning` tab on the left sidebar, and choose `home` as the GoalState. Click on `Plan and Execute` button. This will plan and execute the trajectory to the `home` position, and you should see the arm move to the `home` position.

### MoveIt on the Hardware / Gazebo

The URDF files in the MoveIt config directory does not have access to HEBI Hardware plugin or Gazebo plugins which we defined while setting up ROS2 control URDF files.

Modifying the URDF, SRDF, and launch files can prove to be difficult and hence, we have provided `move_group.launch.py` in the `hebi_bringup` package.

This launch file is supposed to be used in parallel with `bringup_arm` launch files, either directly on the hardware or on Gazebo.

For example, either run
```
ros2 launch hebi_bringup bringup_arm.launch.py hebi_arm:=A-2085-05 use_mock_hardware:=false use_rviz:=false
```
or
```
ros2 launch hebi_bringup bringup_arm_gazebo_classic.launch.py hebi_arm:=A-2085-05 use_rviz:=false
```
or
```
ros2 launch hebi_bringup bringup_arm_gazebo.launch.py hebi_arm:=A-2085-05 use_rviz:=false
```

Then run
```
ros2 launch hebi_bringup move_group.launch.py hebi_arm:=A-2085-05
```

We set `use_rviz` as false in the first launch file to prevent duplicate RViz windows. Once you run the `move_group` launch command, you should see an RViz window with MoveIt loaded.