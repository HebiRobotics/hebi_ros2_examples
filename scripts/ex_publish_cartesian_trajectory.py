#!/usr/bin/env python3

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from hebi_msgs.msg import SE3Trajectory, SE3TrajectoryPoint
from numpy import pi, floor, linspace, nan

class TrajectoryPublisher(Node):
  def __init__(self):
    super().__init__('trajectory_publisher')

    # Define publisher
    self.publisher_ = self.create_publisher(SE3Trajectory, '/cartesian_trajectory', 10)
    
    # Pattern parameters
    self.y_min = -0.15
    self.y_max = 0.15
    self.z_min = 0.1
    self.z_max = 0.3
    self.num_points = 10
    # Initial position
    self.start_position = [0.55, self.y_min, self.z_min, -pi/2, pi/2, 0.0]
    
    # Time variable
    self.t = 0.0
    self.travel_time = 3.0
    self.start_time: int = 2
    
    # Create new SE3Trajectory message
    trajectory_msg = SE3Trajectory()
    trajectory_msg.header.stamp = self.get_clock().now().to_msg()
    trajectory_msg.header.frame_id = 'base_link'

    # Create start point
    point = SE3TrajectoryPoint()
    point.x = self.start_position[0]
    point.y = self.start_position[1]
    point.z = self.start_position[2]
    point.roll = self.start_position[3]
    point.pitch = self.start_position[4]
    point.yaw = self.start_position[5]
    point.time_from_start = Duration(sec=int(floor(self.start_time)), nanosec=int((self.start_time - floor(self.start_time)) * 1e9))
    # Add start point to trajectory
    trajectory_msg.points.append(point)
    
    # Update time variable
    self.t = self.start_time

    # Generate trajectory points for a rectange pattern
    for i in range(self.num_points):
      # Create point for horizontal movement
      y = self.y_min + (self.y_max - self.y_min) * (i / (self.num_points - 1))
      point = SE3TrajectoryPoint()
      point.x = 0.55
      point.y = y
      point.z = self.start_position[2]
      point.roll = self.start_position[3]
      point.pitch = self.start_position[4]
      point.yaw = self.start_position[5]
      point.gripper = 1.0
      self.t += self.travel_time / self.num_points
      point.time_from_start = Duration(sec=int(floor(self.t)), nanosec=int((self.t - floor(self.t)) * 1e9))
      trajectory_msg.points.append(point)

    for i in range(self.num_points):
      # Create point for vertical movement
      z = self.z_min + (self.z_max - self.z_min) * (i / (self.num_points - 1))
      point = SE3TrajectoryPoint()
      point.x = 0.55
      point.y = self.y_max
      point.z = z
      point.roll = self.start_position[3]
      point.pitch = self.start_position[4]
      point.yaw = self.start_position[5]
      point.gripper = 0.0
      self.t += self.travel_time / self.num_points
      point.time_from_start = Duration(sec=int(floor(self.t)), nanosec=int((self.t - floor(self.t)) * 1e9))
      trajectory_msg.points.append(point)

    for i in range(self.num_points):
      # Create point for horizontal movement back to start
      y = self.y_max - (self.y_max - self.y_min) * (i / (self.num_points - 1))
      point = SE3TrajectoryPoint()
      point.x = 0.55
      point.y = y
      point.z = self.z_max
      point.roll = self.start_position[3]
      point.pitch = self.start_position[4]
      point.yaw = self.start_position[5]
      point.gripper = 1.0
      self.t += self.travel_time / self.num_points
      point.time_from_start = Duration(sec=int(floor(self.t)), nanosec=int((self.t - floor(self.t)) * 1e9))
      trajectory_msg.points.append(point)

    for i in range(self.num_points):
      # Create point for vertical movement back to start
      z = self.z_max - (self.z_max - self.z_min) * (i / (self.num_points - 1))
      point = SE3TrajectoryPoint()
      point.x = 0.55
      point.y = self.y_min
      point.z = z
      point.roll = self.start_position[3]
      point.pitch = self.start_position[4]
      point.yaw = self.start_position[5]
      point.gripper = 0.0
      self.t += self.travel_time / self.num_points
      point.time_from_start = Duration(sec=int(floor(self.t)), nanosec=int((self.t - floor(self.t)) * 1e9))
      trajectory_msg.points.append(point)

    # Publish trajectory
    self.publisher_.publish(trajectory_msg)
    self.get_logger().info('Published trajectory')

def main(args=None):
  rclpy.init(args=args)
  trajectory_publisher = TrajectoryPublisher()
  
  try:
    rclpy.spin(trajectory_publisher)
  except KeyboardInterrupt:
    pass
  finally:
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()