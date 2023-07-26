#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically

from rclpy.action import ActionClient
from hebi_msgs.action import ArmMotion
from builtin_interfaces.msg import Duration

from enum import Enum
from numpy import nan
import copy

from time import sleep
import hebi
import numpy as np
import json
import requests


class FSMState(Enum):
  IDLE = 0
  RECORD = 1
  PLAY = 2
  GOTO_WP = 3

class RecordPlayIONode(Node):
  def __init__(self):
    super().__init__('record_play_io_node')

    self.declare_parameter("family", "HEBI")
    self.declare_parameter("name", "mobileIO")
    self.declare_parameter("prefix", "/")
    self.declare_parameter('path_dt', 0.25)

    self.mio = None
    if (not self.initialize()):
      self.get_logger().error("Could not initialize Mobile IO")
      return

    self.timer_mio = self.create_timer(0.02, self.mio_callback)

    # FSM state
    self.fsm_state = FSMState.IDLE
    self.get_logger().info('FSM state set to IDLE')

    self.in_motion = False

    # Create action client for /arm_motion
    self.arm_motion_client = ActionClient(self, ArmMotion, self.get_parameter("prefix").get_parameter_value().string_value + 'arm_motion')

    # Create subscriber for /joint_states
    self.joint_states_subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
    self.joint_state = JointTrajectoryPoint()

    self.joint_trajectory = JointTrajectory()
    self.trajectory_time = 2.0 # seconds
    self.trajectory_dt = 0.25 # seconds

    self.record_timer = self.create_timer(self.trajectory_dt, self.record_trajectory)

    self.waypoints = []
    self.max_wp = 4
  
  def mio_callback(self):
    self.mio.update()

    # self.get_logger().info(",".join([str(self.mio.get_button_diff(x)) for x in range(1, 9)]))

    if self.in_motion:
      self.mio.set_led_color('red')
    else:
      self.mio.set_led_color('green')

    if self.fsm_state == FSMState.IDLE:
      if self.mio.get_button_diff(1) == 1:
        param = Parameter('compliant_mode', Parameter.Type.BOOL, True).to_parameter_msg()
        self.set_external_parameters('arm_node', [param])
        self.fsm_state = FSMState.RECORD
        self.get_logger().info('FSM state set to RECORD')
        self.joint_trajectory = JointTrajectory()
        self.trajectory_time = 2.0
      elif self.mio.get_button_diff(2) == 1:
        param = Parameter('compliant_mode', Parameter.Type.BOOL, False).to_parameter_msg()
        self.set_external_parameters('arm_node', [param])
        self.fsm_state = FSMState.PLAY
        self.get_logger().info('FSM state set to PLAY')
        self.send_trajectory()
      elif self.mio.get_button_diff(3) == 1:
        self.save_waypoint()
      elif self.mio.get_button_diff(4) == 1:
        self.fsm_state = FSMState.GOTO_WP
        self.get_logger().info('FSM state set to GOTO_WP')

    elif self.fsm_state == FSMState.RECORD:
      if self.mio.get_button_diff(1) == -1:
        self.fsm_state = FSMState.IDLE
        self.get_logger().info('FSM state set to IDLE')
        param = Parameter('compliant_mode', Parameter.Type.BOOL, False).to_parameter_msg()
        self.set_external_parameters('arm_node', [param])
      elif self.mio.get_button_diff(3) == 1:
        self.save_waypoint()

    elif self.fsm_state == FSMState.PLAY:
      if self.mio.get_button_diff(1) == 1:
        if not self.in_motion:
          param = Parameter('compliant_mode', Parameter.Type.BOOL, True).to_parameter_msg()
          self.set_external_parameters('arm_node', [param])
          self.fsm_state = FSMState.RECORD
          self.get_logger().info('FSM state set to RECORD')
          self.joint_trajectory = JointTrajectory()
          self.trajectory_time = 2.0
      if self.mio.get_button_diff(2) == 1:
        if not self.in_motion:
          self.send_trajectory()
      elif self.mio.get_button_diff(3) == 1:
        self.save_waypoint()
      elif self.mio.get_button_diff(4) == 1:
        if not self.in_motion:
          self.fsm_state = FSMState.GOTO_WP
          self.get_logger().info('FSM state set to GOTO_WP')

    elif self.fsm_state == FSMState.GOTO_WP:
      if not self.in_motion:
        if self.mio.get_button_diff(1) == 1:
          param = Parameter('compliant_mode', Parameter.Type.BOOL, True).to_parameter_msg()
          self.set_external_parameters('arm_node', [param])
          self.fsm_state = FSMState.RECORD
          self.get_logger().info('FSM state set to RECORD')
          self.joint_trajectory = JointTrajectory()
          self.trajectory_time = 2.0
        elif self.mio.get_button_diff(2) == 1:
          self.fsm_state = FSMState.PLAY
          self.get_logger().info('FSM state set to PLAY')
          self.send_trajectory()
        elif self.mio.get_button_diff(4) == 1:
          self.fsm_state = FSMState.IDLE
          self.get_logger().info('FSM state set to IDLE')
        elif self.mio.get_button_diff(5) == 1:
          self.move_to_waypoint(0)
        elif self.mio.get_button_diff(6) == 1:
          self.move_to_waypoint(1)
        elif self.mio.get_button_diff(7) == 1:
          self.move_to_waypoint(2)
        elif self.mio.get_button_diff(8) == 1:
          self.move_to_waypoint(3)

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

    self.mio.set_button_label(1, "REC")
    self.mio.set_button_label(2, "PLAY")
    self.mio.set_button_label(3, "SAVE_WP")
    self.mio.set_button_label(4, "GOTO_WP")
    self.mio.set_button_label(5, "WP #0")
    self.mio.set_button_label(6, "WP #1")
    self.mio.set_button_label(7, "WP #2")
    self.mio.set_button_label(8, "WP #3")

    self.mio.set_button_mode(1, 1)

    self.mio.set_led_color('green')

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
    self.joint_state.velocities = msg.velocity
    # Zero accelerations since we don't have that information
    self.joint_state.accelerations = [nan] * len(msg.position)
  
  def record_trajectory(self):
    # Check if we are recording
    if self.fsm_state == FSMState.RECORD:
      point = copy.deepcopy(self.joint_state)
      time = self.trajectory_time
      point.time_from_start = Duration(sec=int(time), nanosec=int((time - int(time)) * 1e9))
      self.joint_trajectory.points.append(point)
      self.trajectory_time += self.trajectory_dt

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
    self.in_motion = True

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
    
    self.in_motion = False
    
  def save_waypoint(self):
    if len(self.waypoints) < self.max_wp:
      self.waypoints.append(copy.deepcopy(self.joint_state))
      self.get_logger().info('Waypoint #' + str(len(self.waypoints) - 1) + ' saved')
      return True
    else:
      self.get_logger().error('Max number of waypoints reached')
      return False

  def move_to_waypoint(self, wp_index):
    if wp_index < len(self.waypoints):
      goal_msg = ArmMotion.Goal()
      goal_msg.waypoints.points.append(self.waypoints[wp_index])
      goal_msg.use_wp_times = False
      goal_msg.wp_type = ArmMotion.Goal.JOINT_SPACE
      self.arm_motion_client.wait_for_server()
      self.move_to_waypoint_future = self.arm_motion_client.send_goal_async(goal_msg, feedback_callback=self.send_trajectory_feedback_callback)
      self.move_to_waypoint_future.add_done_callback(self.send_trajectory_response_callback)
      self.get_logger().info('Moving to waypoint #' + str(wp_index))
      return True
    else:
      self.get_logger().error('Invalid waypoint index')
      return False

def main(args=None):
  rclpy.init(args=args)
  node = RecordPlayIONode()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
