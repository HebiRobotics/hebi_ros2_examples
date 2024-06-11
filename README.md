# HEBI ROS2 Packages

## Setting up your HEBI ROS workspace

```
mkdir -p ~/hebi_ws/src
cd ~/hebi_ws/src
git clone -b ros2 https://github.com/HebiRobotics/hebi_cpp_api_ros.git
git clone -b ros2/$ROS_DISTRO https://github.com/HebiRobotics/hebi_description.git # ROS_DISTRO can be humble, iron, or jazzy
git clone https://github.com/HebiRobotics/hebi_msgs.git
git clone https://github.com/HebiRobotics/hebi_ros2_examples.git
```

Optional: If you want HEBI packages for `ros2_control`, execute these too.

```
git clone https://github.com/HebiRobotics/hebi_hardware.git
git clone https://github.com/HebiRobotics/hebi_bringup.git
```

Optional: If you want HEBI package for MoveIt, clone the repos for `ros2_control` and run the following:
```
git clone -b ros2 https://github.com/HebiRobotics/hebi_moveit_configs.git
```

Once you have cloned the necessary repositories, execute the following commands.
```
cd ~/hebi_ws
colcon build
```

---

## Create HRDF

The HRDF for A-2085-05:
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

This step is sufficient if you want to control the arm directly with our `arm_node`. To do so, you need to create a `<your_robot_name>_params.yaml` file in `config` folder of the `hebi_ros2_examples` package.

The configuration file takes the following form:
```
arm_node:
  ros__parameters:
    names:
    families:
    gains_package:
    gains_file:
    hrdf_package:
    hrdf_file:
    home_position:
```

The parameters are defined as follows -
- `names`: Array of "Names" of the HEBI Modules
- `families`: Array of "Family" of HEBI Modules
- `gains_package`: ROS package containing the gains file for your HEBI arm
- `gains_file`: Relative path of the gains file from the gains_package
- `hrdf_package`: ROS package containing the HRDF file for your HEBI arm
- `hrdf_file`: Relative path of the HRDF file from the hrdf_package
- `home_position`: Array of float values to move your arm after initialization.

**NOTE:**
- `names` and `families` of your modules can be found using Scope.
- If the length of `home_position` array is greater than the number of joints, the remaining values will be ignored. **TO BE IMPLEMENTED**

The configuration file for HEBI Arm A-2085-05 is given below.
```
arm_node:
  ros__parameters:
    names: [ "J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2" ]
    families: [ "HEBI" ]
    gains_package: "hebi_description"
    gains_file: "config/gains/A-2085-05_gains.xml"
    hrdf_package: "hebi_description"
    hrdf_file: "config/hrdf/A-2085-05.hrdf"
    home_position: [ 0.01, 2.09, 2.09, 1.5707963, 0.0 ]
```

To run the arm node, execute the following command:
```
ros2 launch hebi_ros2_examples arm_node.launch.py hebi_arm:=<your_robot_name>
```

**NOTE:** Do not forget to build your workspace and source your setup before running the above command.

Read more about `arm_node` in .......................................

## Convert HRDF to URDF

Using the python script given in `hebi_description` package, convert the HRDF to UDRF xacro. The URDF for A-2085-06 looks like this
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

**NOTE-TO-SELF:**
- Script to be checked
- mass of gripper was zero before, it should have some mass

To be compatible with the `hebi_description` package standards, the URDF will be saved as `<robot_name>.urdf.xacro` in the `./urdf/kits` folder.

To check if your arm configuration in RViz, run the following command:
```
ros2 launch hebi_description view_arm.launch.py hebi_arm:=<your_robot_name>
```

**NOTE:** Do not forget to build your workspace and source your setup before running the above command.

If the launch is successful, you should see a RViz window, with the arm and a GUI to control the joints of the arm.

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/hebi_description_1.png)

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/hebi_description_2.png)

## ROS2 Control

If you want to control your arm using `ros2_control`, you will need two files:
- ROS2 Control macro file apart from the URDF file created from the above step.
- Xacro file including the ROS 2 Control macro file and the original URDF file.
- ROS2 Control parameter file

Both of these files should be placed in `urdf/kits/ros2_control` folder of the `hebi_decription` package.

### ROS2 Control Macro File

The template for a HEBI Arm ROS2 Control Macro file is as follows:
```
<?xml version="1.0" encoding="UTF-8"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">

  <xacro:macro name="<your_robot_name>_ros2_control" params="
                name
                use_mock_hardware:=^|false
                mock_sensor_commands:=^|false
                sim_gazebo:=^|false
                families
                names
                hrdf_pkg
                hrdf_file
                gains_pkg
                gains_file"
               >

    <ros2_control name="${name}" type="system">

      <hardware>
        <xacro:if value="${use_mock_hardware}">
          <plugin>mock_components/GenericSystem</plugin>
          <param name="mock_sensor_commands">${mock_sensor_commands}</param>
        </xacro:if>
        <xacro:if value="${sim_gazebo}">
          <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </xacro:if>
        <xacro:unless value="${use_mock_hardware or sim_gazebo}">
          <param name="families">${families}</param>
          <param name="names">${names}</param>
          <param name="hrdf_pkg">${hrdf_pkg}</param>
          <param name="hrdf_file">${hrdf_file}</param>
          <param name="gains_pkg">${gains_pkg}</param>
          <param name="gains_file">${gains_file}</param>
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
    <xacro:if value="$(arg sim_gazebo)">
      <gazebo>
        <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
          <parameters>"<your_controller_param_file>"</parameters>
        </plugin>
      </gazebo>
    </xacro:if>

  </xacro:macro>
</robot>
```

It is recommended to name this file as `<your_robot_name>_macro.ros2_control.xacro`.

The `A-2085-05_macro.ros2_control.xacro` file looks like this: <link_to_github_file>.

### Combined URDF Xacro File

The template for the combined URDF xacro file is given below:
```
<?xml version="1.0" encoding="UTF-8"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="<your_robot_name>">

  <xacro:arg name="families" default="" />
  <xacro:arg name="names" default="" />
  <xacro:arg name="hrdf_pkg" default="" />
  <xacro:arg name="hrdf_file" default="" />
  <xacro:arg name="gains_pkg" default="" />
  <xacro:arg name="gains_file" default="" />

  <xacro:arg name="use_mock_hardware" default="false" />
  <xacro:arg name="mock_sensor_commands" default="false" />
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
    sim_gazebo="$(arg sim_gazebo)"
    families="$(arg families)"
    names="$(arg names)"
    hrdf_pkg="$(arg hrdf_pkg)"
    hrdf_file="$(arg hrdf_file)"
    gains_pkg="$(arg gains_pkg)"
    gains_file="$(arg gains_file)" />

</robot>
```

This file is named similarly to the original URDF file, i.e., `<robot_name>.urdf.xacro`, but saved in the `/urdf/kits/ros2_control` folder.

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
ros2 launch hebi_bringup bringup_arm.launch.py hebi_arm:=<your_robot_name> families:="<string_families_separated_by_semicolon>" names:="<string_names_separated_by_semicolon>"
```
Here is an example for A-2085-05:
```
ros2 launch hebi_bringup bringup_arm.launch.py hebi_arm:=A-2085-05 families:="HEBI" names:="J1_base;J2_shoulder;J3_elbow;J4_wrist1;J5_wrist2"
```
There are many arguments that can be passed to the `bringup_arm.launch.py` file.

To execute the ROS 2 Control node with Gazebo simulation, run the following command:
```
ros2 launch hebi_bringup bringup_arm_gazebo.launch.py hebi_arm:=<your_robot_name>
```
**NOTE:** Do not forget to build your workspace and source your setup before running the above commands.

To test the controllers, you can do so by running the following command:
```
ros2 launch hebi_bringup test_joint_trajectory_controller.launch.py
```

You should see the arm moving according to the joint positions given in the configuration file.

**NOTE: More details**

## MoveIt

If you want to use MoveIt with your HEBI arm, you will need to create a MoveIt configuration package for your arm. The steps to do so are given below.

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

To enable the MoveIt to access the HEBI Hardware Interface, you will need to make the following changes in the `<your_robot_name>_moveit_config` package.

### config/<your_robot_name>.ros2_control.xacro

After the line `
<xacro:property name="initial_positions" value="${load_yaml(initial_positions_file)['initial_positions']}"/>
`, add the following:
```
<xacro:property name="families" value="${load_yaml(initial_positions_file)['families']}"/>
<xacro:property name="names" value="${load_yaml(initial_positions_file)['names']}"/>
<xacro:property name="hrdf_pkg" value="${load_yaml(initial_positions_file)['hrdf_pkg']}"/>
<xacro:property name="hrdf_file" value="${load_yaml(initial_positions_file)['hrdf_file']}"/>
<xacro:property name="gains_pkg" value="${load_yaml(initial_positions_file)['gains_pkg']}"/>
<xacro:property name="gains_file" value="${load_yaml(initial_positions_file)['gains_file']}"/>
<xacro:property name="use_mock_hardware" value="${load_yaml(initial_positions_file)['use_mock_hardware']}"/>
<xacro:property name="use_gazebo" value="${load_yaml(initial_positions_file)['use_gazebo']}"/>
```

Replace the `<hardware>` tag with the following:
```
<hardware>
  <xacro:if value="${use_mock_hardware}">
    <plugin>mock_components/GenericSystem</plugin>
  </xacro:if>
  <xacro:if value="${use_gazebo}">
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </xacro:if>
  <xacro:unless value="${use_mock_hardware or use_gazebo}">
    <!-- Parameters to initialize the Components -->
    <param name="families">${families}</param>
    <param name="names">${names}</param>
    <param name="hrdf_pkg">${hrdf_pkg}</param>
    <param name="hrdf_file">${hrdf_file}</param>
    <param name="gains_pkg">${gains_pkg}</param>
    <param name="gains_file">${gains_file}</param>
    <plugin>hebi_hardware/HEBIHardwareInterface</plugin>
  </xacro:unless>
</hardware>
```

After the `<ros2_control>` tag, add the following:
```
<!-- Gazebo Classic plugins -->
<xacro:if value="${use_gazebo}">
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find hebi_a-2085-06)/config/ros2_controllers.yaml</parameters>
    </plugin>
  </gazebo>
</xacro:if>
```

The final file for A-2085-05 looks like this: <link_to_github_file>

### config/initial_positions.yaml

At the end of the file, add the following:
```
families: "<families_as_string_separated_by_semicolon>"

names: "<names_as_string_separated_by_semicolon>"

hrdf_pkg: "hebi_description"
hrdf_file: "config/hrdf/<your_robot_name>.hrdf"
gains_pkg: "hebi_description"
gains_file: "config/gains/<your_robot_name>_gains.xml"

use_mock_hardware: true
use_gazebo: false
```

To test the MoveIt configuration, run the following command:
```
ros2 launch <your_robot_name>_moveit_config demo.launch.py
```
**NOTE:** Do not forget to build your workspace and source your setup before running the above command.

If the launch is successful, a RViz window opens up with the arm and a GUI to control the arm that looks like this:

![](https://github.com/HebiRobotics/hebi_description/blob/06ab8236d43b2859d3f8ec46a7dd942175c6e785/docs/moveit_1.png)

To check if the MoveIt is able to plan and execute the trajectories, click on the `Planning` tab on the left sidebar, and choose `home` as the GoalState. Click on `Plan and Execute` button. This will plan and execute the trajectory to the `home` position, and you should see the arm move to the `home` position.

To test the MoveIt configuration with the hardware, change the `use_mock_hardware` to `false` in the `initial_positions.yaml` file, and relaunch the `demo.launch.py` file. This will launch the MoveIt with the hardware interface. Now, you can plan and execute the trajectories with the hardware.
**NOTE:** Do not forget to build your workspace and source your setup before relaunching.
