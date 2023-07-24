#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from enum import Enum
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically

from rclpy.action import ActionClient
from hebi_msgs.action import ArmMotion
from builtin_interfaces.msg import Duration

from numpy import nan
import copy


class FSMState(Enum):
  IDLE = 0
  RECORD = 1
  PLAY = 2

class RecordPlayNode(Node):
  def __init__(self):
    super().__init__('record_play_node')

    self.declare_parameter('fsm_state', 'IDLE')

    # FSM state
    self.fsm_state = FSMState(self.get_parameter('fsm_state').get_parameter_value().integer_value)
    self.add_on_set_parameters_callback(self.fsm_state_callback)

    # Create action client for /arm_motion
    self.arm_motion_client = ActionClient(self, ArmMotion, 'arm_motion')

    # Create subscriber for /joint_states
    self.joint_states_subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
    self.joint_state = JointTrajectoryPoint()

    self.joint_trajectory = JointTrajectory()
    self.trajectory_time = 2.0 # seconds
    self.trajectory_dt = 0.25 # seconds

    self.record_timer = self.create_timer(self.trajectory_dt, self.record_trajectory)
  
  def set_external_parameters(self, node_name, parameters):
    request = SetParametersAtomically.Request()
    request.parameters = parameters
    client = self.create_client(SetParametersAtomically, '/' + node_name + '/set_parameters_atomically')
    while not client.wait_for_service(timeout_sec=1.0):
      if not rclpy.ok():
        self.get_logger().error('Interrupted while waiting for the service. Exiting!')
      self.get_logger().info('service not available, waiting again...')

    future = client.call_async(request)

    if future.done():
      try:
        response = future.result()
        self.get_logger().info('Service call success %r' % (response,))
      except Exception as e:
        self.get_logger().error('Service call failed %r' % (e,))
    
    return
  
  def fsm_state_callback(self, parameters):
    fsm_param = None
    for param in parameters:
      if param.name == 'fsm_state':
        fsm_param = param
        break
    
    if fsm_param.type_ != Parameter.Type.STRING:
      self.get_logger().error('Invalid parameter type')
      return SetParametersResult(successful=False)

    if fsm_param.value == 'IDLE':
      self.fsm_state = FSMState.IDLE
      self.get_logger().info('FSM state set to IDLE')
    elif fsm_param.value == 'RECORD':
      # Check if we are transitioning from PLAY to RECORD
      if self.fsm_state == FSMState.PLAY:
        self.get_logger().error('Invalid FSM transition')
        return SetParametersResult(successful=False)
      else:
        param = Parameter('compliant_mode', Parameter.Type.BOOL, True).to_parameter_msg()
        self.set_external_parameters('arm_node', [param])
        self.fsm_state = FSMState.RECORD
        self.get_logger().info('FSM state set to RECORD')
        self.joint_trajectory = JointTrajectory()
        self.trajectory_time = 2.0
    elif fsm_param.value == 'PLAY':
      # Check if we are transitioning from RECORD to PLAY
      if self.fsm_state == FSMState.RECORD:
        self.get_logger().error('Invalid FSM transition')
        return SetParametersResult(successful=False)
      else:
        param = Parameter('compliant_mode', Parameter.Type.BOOL, False).to_parameter_msg()
        self.set_external_parameters('arm_node', [param])
        self.fsm_state = FSMState.PLAY
        self.get_logger().info('FSM state set to PLAY')
        self.send_trajectory()
    else:
      self.get_logger().error('Invalid FSM state')
      return SetParametersResult(successful=False)
    
    return SetParametersResult(successful=True)
  
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
    goal_msg.wp_type = 'JOINT'
    self.arm_motion_client.wait_for_server()
    self.send_trajectory_future = self.arm_motion_client.send_goal_async(goal_msg, feedback_callback=self.send_trajectory_feedback_callback)
    self.send_trajectory_future.add_done_callback(self.send_trajectory_response_callback)
  
  def send_trajectory_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().error('Trajectory rejected by server')
      return
    
    self.get_logger().info('Trajectory accepted by server, executing...')

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
    
    # set parameter to IDLE
    # self.set_parameters([Parameter('fsm_state', Parameter.Type.STRING, 'IDLE')])
    
    

def main(args=None):
  rclpy.init(args=args)
  node = RecordPlayNode()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
