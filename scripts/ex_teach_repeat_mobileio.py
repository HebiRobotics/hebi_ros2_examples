#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically

from rclpy.action import ActionClient
from hebi_msgs.action import ArmMotion

from numpy import nan
import copy

from time import sleep
import hebi

from builtin_interfaces.msg import Duration

def add_durations(a: Duration, b: Duration) -> Duration:
  c = Duration()
  c.sec = a.sec + b.sec
  c.nanosec = a.nanosec + b.nanosec
  if c.nanosec >= 1e9:
    c.sec += 1
    c.nanosec -= 1e9
  return c


class TeachRepeatIONode(Node):
  def __init__(self):
    super().__init__('teach_repeat_io_node')

    self.declare_parameter("family", "HEBI")
    self.declare_parameter("name", "mobileIO")
    self.declare_parameter("prefix", "/")

    self.mio = None
    if (not self.initialize()):
      self.get_logger().error("Could not initialize Mobile IO")
      return

    self.timer_mio = self.create_timer(0.02, self.mio_callback)

    self.mode = "training"

    # Create action client for /arm_motion
    self.arm_motion_client = ActionClient(self, ArmMotion, self.get_parameter("prefix").get_parameter_value().string_value + 'arm_motion')

    # Create subscriber for /joint_states
    self.joint_states_subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
    self.joint_state = JointTrajectoryPoint()

    self.joint_trajectory = JointTrajectory()
    self.time_between_waypoints = Duration(sec=0)
  
  def mio_callback(self):
    self.mio.update()

    time_wp = self.mio.get_axis_state(3) + 3.0
    # Split into seconds and nanoseconds
    self.time_between_waypoints.sec = int(time_wp)
    self.time_between_waypoints.nanosec = int((time_wp - self.time_between_waypoints.sec) * 1e9)

    if self.mode == "training":
      # B1 - add waypoint (stop)
      if self.mio.get_button_diff(1) == 1:
        point = copy.deepcopy(self.joint_state)
        if len(self.joint_trajectory.points) == 0:
          point.time_from_start = self.time_between_waypoints
        else:
          point.time_from_start = add_durations(self.joint_trajectory.points[-1].time_from_start, self.time_between_waypoints)
        point.velocities = [0.0] * len(point.positions)
        self.joint_trajectory.points.append(point)
        self.get_logger().info('Stop Waypoint Added')
      
      # B2 - add waypoint (flow)
      elif self.mio.get_button_diff(2) == 1:
        point = copy.deepcopy(self.joint_state)
        if len(self.joint_trajectory.points) == 0:
          point.time_from_start = self.time_between_waypoints
          point.velocities = [0.0] * len(point.positions)
        else:
          point.time_from_start = add_durations(self.joint_trajectory.points[-1].time_from_start, self.time_between_waypoints)
        self.joint_trajectory.points.append(point)
        self.get_logger().info('Flow Waypoint Added')

      # B3 - toggle training/playback
      elif self.mio.get_button_diff(3) == 1:
        param = Parameter('compliant_mode', Parameter.Type.BOOL, False).to_parameter_msg()
        self.set_external_parameters('arm_node', [param])
        if len(self.joint_trajectory.points) > 1:
          self.mode = "playback"
          self.get_logger().info('Switching to playback mode')
          self.mio.set_button_label(3, "TRAIN")
          self.send_trajectory()
      
      # B4 - clear waypoints
      elif self.mio.get_button_diff(4) == 1:
        self.joint_trajectory = JointTrajectory()
        self.get_logger().info('Waypoints cleared')
    
    elif self.mode == "playback":
      if self.mio.get_button_diff(3) == 1:
        self.mode = "training"
        self.get_logger().info('Switching to training mode')
        self.mio.set_led_color('blue')
        param = Parameter('compliant_mode', Parameter.Type.BOOL, True).to_parameter_msg()
        self.set_external_parameters('arm_node', [param])
        self.mio.set_button_label(3, "PLAY")

  def initialize(self):
    self.get_logger().info("Initializing Mobile IO...")
    
    lookup = hebi.Lookup()
    sleep(1)

    family = self.get_parameter("family").get_parameter_value().string_value
    name = self.get_parameter("name").get_parameter_value().string_value

    self.get_logger().info("Connecting to Mobile IO with family: \"" + family + "\" and name: \"" + name + "\"")
    self.mio = hebi.util.create_mobile_io(lookup, family, name)
    if self.mio is None:
        self.get_logger().error("Could not find Mobile IO")
        return False
    
    self.get_logger().info("Connected to Mobile IO!")

    self.mio.resetUI()
    

    self.mio.set_button_label(1, "ADD WP\n(STOP)")
    self.mio.set_button_label(2, "ADD WP\n(FLOW)")
    self.mio.set_button_label(3, "PLAY")
    self.mio.set_button_label(4, "CLEAR WP")

    # Print Instructions
    instructions = "\n"
    instructions += """B1 - Add waypoint (stop)
    B2 - Add waypoint (flow)
    A3 - Up/down for longer/shorter time to waypoint
    B3 - Toggle training/playback
    B4 - Clear waypoints
    B8 - Quit
    """
    self.get_logger().info(instructions)
    self.mio.clear_text()
    self.mio.add_text(instructions)
    self.mio.set_led_color('blue')

    param = Parameter('compliant_mode', Parameter.Type.BOOL, True).to_parameter_msg()
    self.set_external_parameters('arm_node', [param])

    return True

  def set_external_parameters(self, node_name, parameters):
    request = SetParametersAtomically.Request()
    request.parameters = parameters
    client = self.create_client(SetParametersAtomically, '/' + node_name + '/set_parameters_atomically')

    if not client.wait_for_service(timeout_sec=1.0):
      self.get_logger().error('Service not available.')
      
    future = client.call_async(request)

    if future.done():
      try:
        response = future.result()
        self.get_logger().info('Service call success %r' % (response,))
      except Exception as e:
        self.get_logger().error('Service call failed %r' % (e,))
  
  def joint_states_callback(self, msg):
    self.joint_state.positions = msg.position
    self.joint_state.velocities = [nan] * len(msg.position)
    self.joint_state.accelerations = [nan] * len(msg.position)

  def send_trajectory(self):
    goal_msg = ArmMotion.Goal()
    goal_msg.waypoints = self.joint_trajectory
    goal_msg.use_wp_times = True
    goal_msg.wp_type = ArmMotion.Goal.JOINT_SPACE
    self.arm_motion_client.wait_for_server()
    self.send_trajectory_future = self.arm_motion_client.send_goal_async(goal_msg, feedback_callback=self.send_trajectory_feedback_callback)
    self.send_trajectory_future.add_done_callback(self.send_trajectory_response_callback)
  
  def send_trajectory_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().error('Trajectory rejected by server')
      return
    
    self.get_logger().info('Trajectory accepted by server, executing...')
    self.mio.set_led_color('green')

    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.send_trajectory_done_callback)
  
  def send_trajectory_feedback_callback(self, feedback_msg):
    self.get_logger().info('Percent Complete: {0}'.format(feedback_msg.feedback.percent_complete))
  
  def send_trajectory_done_callback(self, future):
    result = future.result().result
    if result.success:
      self.get_logger().info('Trajectory successfully executed')
    else:
      self.get_logger().error('Trajectory execution failed')
    
    self.mio.set_led_color('blue')


def main(args=None):
  rclpy.init(args=args)
  node = TeachRepeatIONode()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
