#!/usr/bin/env python3

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from numpy import pi, floor, sin, cos, nan


class TrajectoryPublisher(Node):
  def __init__(self):
    super().__init__('trajectory_publisher')
    self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
    self.dt = 0.1
    self.t = 1.0

    use_gripper = False

    trajectory_msg = JointTrajectory()
    trajectory_msg.header.frame_id = 'base_link'
    trajectory_msg.joint_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']

    point = JointTrajectoryPoint()
    point.positions = [0.0, 1.2, 1.8, 0.6, 1.5708, 0.0] + ([0.0] if use_gripper else [])
    point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + ([0.0] if use_gripper else [])
    point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + ([0.0] if use_gripper else [])
    point.time_from_start = Duration(sec=int(floor(self.t)), nanosec=int((self.t-floor(self.t))*1e9))
    trajectory_msg.points.append(point)

    while (self.t <= 13.0):
      self.t += self.dt
      point = JointTrajectoryPoint()
      point.positions = [pi/4 * sin(pi*(self.t-1.0)/4), 1.2, 1.8, 0.6, 1.5708, 0.0] + ([sin(pi*(self.t-1.0)/4)] if use_gripper else [])
      point.velocities = [pi**2/16 * cos(pi*(self.t-1.0)/4), 0.0, 0.0, 0.0, 0.0, 0.0] + ([0.0] if use_gripper else [])
      point.accelerations = [nan]*6 + ([nan] if use_gripper else [])
      point.time_from_start = Duration(sec=int(floor(self.t)), nanosec=int((self.t-floor(self.t))*1e9))
      trajectory_msg.points.append(point)

    self.get_logger().info('Published trajectory')
    self.publisher_.publish(trajectory_msg)

def main(args=None):
  rclpy.init(args=args)
  trajectory_publisher = TrajectoryPublisher()
  rclpy.spin(trajectory_publisher)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
