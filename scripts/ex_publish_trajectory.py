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
    self.dt = 0.25
    self.t = 1.0

    trajectory_msg = JointTrajectory()
    trajectory_msg.header.frame_id = 'base_link'
    trajectory_msg.joint_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3']

    point = JointTrajectoryPoint()
    point.positions = [0.0, 2.09439, 2.09439, 0.0, pi/2, 0.0]
    point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    point.time_from_start = Duration(sec=int(floor(self.t)), nanosec=int((self.t-floor(self.t))*1e9))
    trajectory_msg.points.append(point)

    while (self.t <= 12.0):
      self.t += self.dt
      point = JointTrajectoryPoint()
      point.positions = [pi/2 * sin(pi*(self.t-1.0)/4), 2.09439, 2.09439, 0.0, pi/2, 0.0]
      point.velocities = [pi**2/8 * cos(pi*(self.t-1.0)/4), 0.0, 0.0, 0.0, 0.0, 0.0]
      point.accelerations = [nan]*6
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
