#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
import std_msgs
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urllib.parse import urlparse
from hebi_msgs.srv import SetGainsFile 

import hebi
from hebi._internal.errors import HEBI_Exception
import math
import numpy as np
import os

class GroupNode(Node):
    def __init__(self):
        super().__init__('group_node')

    def initialize(self):
        # Get fundamental names/families parameters
        self.declare_parameter('families', ["HEBI"])
        families = self.get_parameter('families').get_parameter_value().string_array_value
        
        self.declare_parameter('names', value=None, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY, description='Required parameter for names of modules'))
        try:
            names = self.get_parameter('names').get_parameter_value().string_array_value
        except ParameterUninitializedException:
            self.get_logger().error('Could not find/read required \'names\' parameter; aborting!')
            return False

        # Find the required group object (or fail)
        lookup = hebi.Lookup()
        for num_tries in range(3):
            self.group = lookup.get_group_from_names(families, names, timeout_ms=2500)
            if self.group:
                break
            if num_tries < 2:
                self.get_logger().warn(f'Could not find group actuators, trying again ({num_tries + 1}/2 retries)')
            lookup.reset()

        if self.group is None:
            return False

        # Read package and path for gains, and set if given
        self.declare_parameter('gains_package', "")
        gains_package = self.get_parameter('gains_package').get_parameter_value().string_value
        self.declare_parameter('gains_file', "")
        gains_file = self.get_parameter('gains_file').get_parameter_value().string_value
        if len(gains_package) > 0 and len(gains_file) > 0:
            self.set_gains(gains_package, gains_file)

        # Read/set dynamic parameters

        # Declare and read command lifetime parameter, using existing value as default (note API value is ms, and param is seconds)
        self.declare_parameter('command_lifetime', self.group.command_lifetime / 1000)
        self.group.command_lifetime = self.get_parameter('command_lifetime').get_parameter_value().double_value * 1000
        # Declare and read feedback frequency parameter, using existing value as default
        self.declare_parameter('feedback_frequency', self.group.feedback_frequency)
        self.group.feedback_frequency = self.get_parameter('feedback_frequency').get_parameter_value().double_value
        # Declare message timeout parameter for length of time to wait for new setpoint message before clearing goal
        self.declare_parameter('message_timeout', 0.0)
        self.message_timeout = self.get_parameter('message_timeout').get_parameter_value().double_value

        # Register the dynamic parameter callback
        self.add_on_set_parameters_callback(self.param_callback) 

        # Set up bookkeeping and cached API objects
        self.trajectory = None
        self.trajectory_start_time = math.nan
        self.last_time = 0
        self.last_setpoint_time = 0
        self.message_cleared = False
        self.command = hebi.GroupCommand(self.group.size)
        self.feedback = hebi.GroupFeedback(self.group.size)

        # Set up publisher and message
        self.group_state_pub = self.create_publisher(JointState, 'joint_states', 50)
        self.state_msg = JointState()
        if len(names) == len(families):
            self.state_msg.name = [f'{families[i]}/{names[i]}' for i in range(len(names))]
        elif len(families) == 1:
            self.state_msg.name = [f'{families[0]}/{n}' for n in names]
        elif len(names) == 1:
            self.state_msg.name = [f'{f}/{names[0]}' for f in families]

        # Set up subscribers
        self.joint_trajectory_sub = self.create_subscription(JointTrajectory, 'joint_waypoints', self.update_joint_waypoints, 50)
        self.joint_point_sub = self.create_subscription(JointTrajectoryPoint, 'joint_target', self.set_joint_setpoint, 50)

        # Set up services
        self.srv_set_gains = self.create_service(SetGainsFile, 'set_gains_file', self.set_gains_callback)
        return True

    def set_gains(self, package: str, file: str):
        gains_cmd = hebi.GroupCommand(self.group.size)
        try:
            package_path = get_package_share_directory(package)
            gains_cmd.read_gains(os.path.join(package_path, file))
        except PackageNotFoundError:
            self.get_logger().error(f'Could not get package path for {package} when loading gains')
            return False
        except HEBI_Exception:
            self.get_logger().error(f'Could not load gains file {file} in path {package_path}')
            return False

        if not self.group.send_command_with_acknowledgement(gains_cmd):
            self.get_logger().error('Could not set group gains')
        else:
            return True

        return False

    def set_gains_callback(self, request, response):
        try:
            # Parse package:// URI
            parsed_uri = urlparse(request.gains_file_uri)
            if parsed_uri.scheme != 'package':
                raise ValueError(f"Unsupported URI scheme: {parsed_uri.scheme}")
            package_name = parsed_uri.netloc
            relative_path = parsed_uri.path.lstrip('/')

            # Try to set the gains
            success = self.set_gains(package_name, relative_path)
            if success:
                response.success = True
            else:
                response.success = False

        except Exception as e:
            self.get_logger().error(f'Failed to set gains: {e}')
            response.success = False

        return response

    def param_callback(self, params):
        result = SetParametersResult(successful=True)
        
        for param in params:
            if param.name == 'command_lifetime':
                if param.type_ == Parameter.Type.DOUBLE:
                    if param.value < 0.0:
                        result.successful = False
                        result.reason = 'Command Lifetime must not be less than 0.0!'
                    else:
                        self.group.command_lifetime = param.value * 1000
                        self.get_logger().info(f'Dynamically updated command lifetime to: {self.group.command_lifetime / 1000}')
                else:
                    result.successful = False
                    result.reason = 'command_lifetime must be a double!'
            elif param.name == 'feedback_frequency':
                if param.type_ == Parameter.Type.DOUBLE:
                    if param.value < 0.0:
                        result.successful = False
                        result.reason = 'Feedback Frequency must not be less than 0.0!'
                    else:
                        self.group.feedback_frequency = param.value
                        self.get_logger().info(f'Dynamically updated feedback frequency to: {self.group.feedback_frequency}')
                else:
                    result.successful = False
                    result.reason = 'feedback_frequency must be a double!'
                    
        return result

    def update(self):
        t = self.get_clock().now().nanoseconds / 1e9
        if t < self.last_time:
            return False

        self.last_time = t
    
        if self.group.get_next_feedback(reuse_fbk=self.feedback) is None:
            return False

        # Update command from trajectory
        if self.trajectory is not None:
            # (trajectory_start_time should not be nan here!)
            t_traj = self.get_clock().now().nanoseconds / 1e9 - self.trajectory_start_time
            t_traj = min(t_traj, self.trajectory.duration)
            [pos, vel, accel] = self.trajectory.get_state(t_traj)

            self.command.position = pos
            self.command.velocity = vel
        else:
            if not self.message_cleared and (self.message_timeout > 0) and (t > (self.last_setpoint_time + self.message_timeout)):
                self.message_cleared = True
                self.get_logger().warn('No setpoint message received within message_timeout, clearing setpoint.')
                clear = [math.nan] * self.command.size()
                self.command.position = clear
                self.command.velocity = clear
                self.command.effort = clear
        if not self.group.send_command(self.command):
            return False

        self.publish_state()
        return True

    def publish_state(self):
        self.state_msg.position = list(self.feedback.position)
        self.state_msg.velocity = list(self.feedback.velocity.astype(np.float64))
        self.state_msg.effort = list(self.feedback.effort.astype(np.float64))
        self.state_msg.header.stamp = self.get_clock().now().to_msg()
        self.group_state_pub.publish(self.state_msg)

    def update_joint_waypoints(self, joint_trajectory: JointTrajectory):
        num_joints = self.group.size
        num_waypoints = len(joint_trajectory.points)

        if num_waypoints == 0:
            self.get_logger().error(f'No waypoints provided!')
            return

        pos = np.ndarray([num_joints, num_waypoints], np.float64)
        vel = np.ndarray([num_joints, num_waypoints], np.float64)
        acc = np.ndarray([num_joints, num_waypoints], np.float64)
        times = np.ndarray([num_waypoints], np.float64)

        for idx, wp in enumerate(joint_trajectory.points):
            if len(wp.positions) != num_joints or len(wp.velocities) != num_joints or len(wp.accelerations) != num_joints:
                self.get_logger().error(f'Position, velocity, or acceleration sizes not correct for waypoint index {idx}')
                return

            if len(wp.effort) != 0:
                self.get_logger().warn('effort commands in trajectories not supported; ignoring')

            pos[:, idx] = wp.positions
            vel[:, idx] = wp.velocities
            acc[:, idx] = wp.accelerations

            times[idx] = wp.time_from_start.sec + wp.time_from_start.nanosec / 1e9

        # If there is a current trajectory, use the commands as a starting point;
        # if not, replan from current feedback.
        if self.trajectory:
            t_traj = self.last_time - self.trajectory_start_time
            t_traj = min(t_traj, self.trajectory.duration)
            [curr_pos, curr_vel, curr_acc] = self.trajectory.get_state(t_traj)
        else:
            curr_pos = self.feedback.position
            curr_vel = self.feedback.velocity
            # (accelerations remain zero)
            curr_acc = np.zeros(self.group.size, dtype=np.float64)

        new_waypoints = num_waypoints + 1
        new_pos = np.ndarray([num_joints, new_waypoints], np.float64)
        new_vel = np.ndarray([num_joints, new_waypoints], np.float64)
        new_acc = np.ndarray([num_joints, new_waypoints], np.float64)

        # Initial state
        new_pos[:, 0] = curr_pos
        new_vel[:, 0] = curr_vel
        new_acc[:, 0] = curr_acc

        # Copy new waypoints
        new_pos[:, 1:] = pos
        new_vel[:, 1:] = vel
        new_acc[:, 1:] = acc

        waypoint_times = np.ndarray([new_waypoints], np.float64)
        waypoint_times[0] = 0
        waypoint_times[1:] = times

        # Create new trajectory
        self.get_logger().info('Creating new trajectory')
        self.trajectory = hebi.trajectory.create_trajectory(waypoint_times, new_pos, new_vel, new_acc)
        self.trajectory_start_time = self.last_time

    def set_joint_setpoint(self, joint_setpoint: JointTrajectoryPoint):
        if self.trajectory:
            self.get_logger().info('Cancelling trajectory, switching to setpoint control')
        self.trajectory = None
        self.trajectory_start_time = math.nan
        # record time of last received setpoint (used to clear setpoint if message_timeout is set)
        self.message_cleared = False
        self.last_setpoint_time = self.get_clock().now().nanoseconds / 1e9

        self.command.position = joint_setpoint.positions
        self.command.velocity = joint_setpoint.velocities
        self.command.effort = joint_setpoint.effort

def main(args=None):
    rclpy.init(args=args)
    node = GroupNode()

    if not node.initialize():
        node.get_logger().error(f'Could not initialize group! Check for modules on the network and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...')
        return

    try:
        while rclpy.ok():
            # Update feedback, and command the arm to move along its planned path
            # (this also acts as a loop-rate limiter so no 'sleep' is needed)
            # Publish any available feedback at each timer tick
            if not node.update():
                node.get_logger().warn('Error Getting Feedback/Sending Commands -- Check Connection')
            # Call any pending callbacks (note -- this may update our planned motion)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()