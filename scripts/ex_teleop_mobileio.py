#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically

from control_msgs.msg import JointJog
from rclpy.action import ActionClient
from hebi_msgs.action import ArmMotion
from std_srvs.srv import Empty

from time import sleep
import hebi


class TeleopNode(Node):
  def __init__(self):
    super().__init__('teleop_node')

    self.declare_parameter("family", "HEBI")
    self.declare_parameter("name", "mobileIO")
    self.declare_parameter("prefix", "/")

    self.home_client = self.create_client(Empty, '/home')

    self.mio = None
    if (not self.initialize()):
      self.get_logger().error("Could not initialize Mobile IO")
      return

    self.timer_mio = self.create_timer(0.02, self.mio_callback)

    # Create publisher for /se3_jog
    self.jog_publisher = self.create_publisher(JointJog, '/SE3_jog', 10)
    # Create action client for /arm_motion
    self.arm_motion_client = ActionClient(self, ArmMotion, self.get_parameter("prefix").get_parameter_value().string_value + 'arm_motion')

    self.last_btn_pressed_time = time.time()

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

    self.mio.set_button_label(1, '⟲', blocking=False)
    self.mio.set_button_label(2, 'Compliant', blocking=False)
    self.mio.set_button_mode(2, 1)
    self.mio.set_button_label(3, ' ', blocking=False)
    self.mio.set_button_label(4, ' ', blocking=False)
    self.mio.set_button_label(5, ' ', blocking=False)
    self.mio.set_button_label(6, ' ', blocking=False)
    self.mio.set_button_label(7, ' ', blocking=False)
    self.mio.set_button_label(8, ' ', blocking=False)

    self.mio.set_axis_label(3, ' ', blocking=False)
    self.mio.set_axis_label(4, ' ', blocking=False)
    self.mio.set_axis_label(5, ' ', blocking=False)
    self.mio.set_axis_label(6, 'z', blocking=False)

    self.mio.set_axis_label(1, ' ')
    self.mio.set_axis_label(7, ' ')
    self.mio.set_axis_label(2, 'rotate')
    self.mio.set_axis_label(8, 'x, y')
    self.mio.set_axis_label(3, 'wrist', blocking=False)
    self.mio.set_snap(3, 0)
    self.mio.set_snap(6, 0)

    self.mio.set_led_color('blue')

    return True

  def mio_callback(self):
    self.mio.update()

    if self.mio.get_button_diff(2):
      if (time.time() - self.last_btn_pressed_time) < 0.5:
        return
      self.last_btn_pressed_time = time.time()
      param = Parameter('compliant_mode', Parameter.Type.BOOL, self.mio.get_button_state(2)).to_parameter_msg()
      print('Setting compliant mode to', self.mio.get_button_state(2))
      self.set_external_parameters('arm_node', [param])
    elif self.mio.get_button_state(1):
      if (time.time() - self.last_btn_pressed_time) < 0.5:
        return
      self.last_btn_pressed_time = time.time()
      self.home()
    else:
      arm_dx = 0.1 * self.mio.get_axis_state(8)
      arm_dy = -0.1 * self.mio.get_axis_state(7)
      arm_dz = 0.1 * self.mio.get_axis_state(6)

      arm_drx = 0.5 * self.mio.get_axis_state(3)
      arm_dry = 0.5 * self.mio.get_axis_state(2)
      arm_drz = 0.5 * self.mio.get_axis_state(1)

      if abs(arm_dx) > 0.01 or abs(arm_dy) > 0.01 or abs(arm_dz) > 0.01 or abs(arm_drx) > 0.01 or abs(arm_dry) > 0.01 or abs(arm_drz) > 0.01:
        jog_msg = JointJog()
        jog_msg.header.stamp = self.get_clock().now().to_msg()
        jog_msg.duration = 0.3
        jog_msg.displacements = [arm_dx, arm_dy, arm_dz, arm_drx, arm_dry, arm_drz]

        self.jog_publisher.publish(jog_msg)

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
  
  def home(self):
    self.get_logger().info('Homing...')
    tryout_times = 3
    while not self.home_client.wait_for_service(timeout_sec=1.0) and tryout_times > 0:
      self.get_logger().info('Service not available, waiting again...')
      tryout_times -= 1

    if tryout_times == 0:
      self.get_logger().error('Service not available.')
      return
    
    request = Empty.Request()
    future = self.home_client.call_async(request)
    future.add_done_callback(self.home_done_callback)

  def home_done_callback(self, future):
    if future.result() is not None:
      self.get_logger().info('Home service success')
    else:
      self.get_logger().error('Home service failed')
  
  def send_trajectory_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().error('Trajectory rejected by server')
      return
    
    self.get_logger().info('Trajectory accepted by server, executing...')
    self.mio.set_led_color('green')

    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.send_trajectory_done_callback)
  
  def send_trajectory_done_callback(self, future):
    result = future.result().result
    if result.success:
      self.get_logger().info('Trajectory successfully executed')
    else:
      self.get_logger().error('Couuld not execute trajectory')
    
    self.mio.set_led_color('blue')


def main(args=None):
  rclpy.init(args=args)
  node = TeleopNode()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
