#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import JointJog
from geometry_msgs.msg import Twist
from time import sleep
import hebi
import numpy as np
import json
import requests


class RosieIONode(Node):
    def __init__(self):
        super().__init__('rosie_io_node')

        self.declare_parameter("family", "HEBI")
        self.declare_parameter("name", "mobileIO")
        self.declare_parameter("prefix", "arm")
        self.declare_parameter("layout_file", "mobileIO_layout.json")

        self.mio = None
        self.timer_mio = self.create_timer(0.02, self.mio_callback)
        
        # Create publishers for the /cartesian_jog and /joint_jog topics
        self.cartesian_jog_pub = self.create_publisher(JointJog, self.get_parameter("prefix").get_parameter_value().string_value + '/cartesian_jog', 10)
        self.joint_jog_pub = self.create_publisher(JointJog, self.get_parameter("prefix").get_parameter_value().string_value + '/joint_jog', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.num_joints = 5
        self.joint_names = ['J1_base', 'J2_shoulder', 'J3_elbow', 'J4_wrist1', 'J5_wrist2']#, 'J6_wrist3']

        self.joint_dt = 0.2
        self.joint_velocity_max = 1.0
        self.joint_velocity = np.zeros(self.num_joints)
        self.prev_joint_velocity = np.zeros(self.num_joints)
        self.joint_direction = 1.0
        self.alpha_joint = 0.1

        self.cartesian_dt = 0.2
        self.cartesian_velocity_max = 0.25
        self.cartesian_velocity = np.zeros(3)
        self.prev_cartesian_velocity = np.zeros(3)
        self.alpha_cartesian = 0.2

        self.cmd_vel_linear_max = 0.5
        self.cmd_vel_angular_max = np.pi/2
        self.cmd_vel_linear = np.zeros(2)
        self.cmd_vel_angular = 0.0

        self.prev_cmd_vel_linear = np.zeros(2)
        self.prev_cmd_vel_angular = 0.0
        self.alpha_cmd_vel = 0.1

        if (not self.initialize()):
            self.get_logger().error("Could not initialize Mobile IO")
            return

        self.timer_cartesian_jog = self.create_timer(0.05, self.cartesian_jog_callback)
        self.timer_joint_jog = self.create_timer(0.05, self.joint_jog_callback)
        self.timer_cmd_vel = self.create_timer(0.05, self.cmd_vel_callback)
    
    def mio_callback(self):
        self.mio.update()

        self.joint_direction = -(int(self.mio.get_button_state(7)) * 2 - 1)

        for i in range(self.num_joints):
            self.joint_velocity[i] = int(self.mio.get_button_state(i+1)) * self.joint_velocity_max * self.joint_direction

        self.cartesian_velocity[0] = self.mio.get_axis_state(8) * self.cartesian_velocity_max
        self.cartesian_velocity[1] = -self.mio.get_axis_state(7) * self.cartesian_velocity_max
        self.cartesian_velocity[2] = self.mio.get_axis_state(5) * self.cartesian_velocity_max

        self.cmd_vel_linear[0] = self.mio.get_axis_state(2)
        self.cmd_vel_linear[1] = -self.mio.get_axis_state(1)
        if np.linalg.norm(self.cmd_vel_linear) > 1.0:
            self.cmd_vel_linear = self.cmd_vel_linear / np.linalg.norm(self.cmd_vel_linear)
        self.cmd_vel_linear = self.cmd_vel_linear * self.cmd_vel_linear_max

        self.cmd_vel_angular = self.mio.get_axis_state(3) * self.cmd_vel_angular_max

    def cartesian_jog_callback(self):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.joint_names = ['x', 'y', 'z']
        msg.duration = self.cartesian_dt

        self.prev_cartesian_velocity = self.cartesian_velocity * self.alpha_cartesian + self.prev_cartesian_velocity * (1.0 - self.alpha_cartesian)
        
        msg.displacements = (self.prev_cartesian_velocity * self.cartesian_dt).tolist()

        # Don't publish if no joints are moving
        if np.linalg.norm(self.prev_cartesian_velocity) > 1e-3:
            self.cartesian_jog_pub.publish(msg)

    def joint_jog_callback(self):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.joint_names = self.joint_names
        msg.duration = self.joint_dt

        self.prev_joint_velocity = self.joint_velocity * self.alpha_joint + self.prev_joint_velocity * (1.0 - self.alpha_joint)

        msg.displacements = (self.prev_joint_velocity * self.joint_dt).tolist()
        msg.velocities = self.prev_joint_velocity.tolist()

        # Don't publish if no joints are moving
        if np.linalg.norm(self.prev_joint_velocity) > 1e-2:
            self.joint_jog_pub.publish(msg)
    
    def cmd_vel_callback(self):
        msg = Twist()
        msg.linear.x = self.cmd_vel_linear[0] * self.alpha_cmd_vel + self.prev_cmd_vel_linear[0] * (1.0 - self.alpha_cmd_vel)
        msg.linear.y = self.cmd_vel_linear[1] * self.alpha_cmd_vel + self.prev_cmd_vel_linear[1] * (1.0 - self.alpha_cmd_vel)
        msg.angular.z = self.cmd_vel_angular * self.alpha_cmd_vel + self.prev_cmd_vel_angular * (1.0 - self.alpha_cmd_vel)

        self.cmd_vel_pub.publish(msg)

        self.prev_cmd_vel_linear = [msg.linear.x, msg.linear.y]
        self.prev_cmd_vel_angular = msg.angular.z


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

        # Commented out until the MobileIO app is fixed
        
        # layout_file = self.get_parameter("layout_file").get_parameter_value().string_value
        # layout_file_path = get_package_share_directory('hebi_ros2_examples') + "/config/" + layout_file

        # self.get_logger().info("Loading layout file: " + layout_file_path)

        # layout_url = 'http://10.10.10.150/'
        # with open(layout_file_path) as f:
        #     layout = json.load(f)
        # response = requests.post(layout_url, json=layout, timeout=5)
        # if response.status_code != 200:
        #     self.get_logger().error("Could not load layout file")
        #     return False
        
        # self.get_logger().info("Loaded layout file!")

        self.mio.set_axis_label(1, "vy")
        self.mio.set_axis_label(2, "vx")
        self.mio.set_axis_label(7, "y")
        self.mio.set_axis_label(8, "x")

        self.mio.set_button_mode(7, 1)

        self.mio.set_snap(3, 0.0)
        self.mio.set_snap(5, 0.0)
        self.mio.set_led_color('green')

        return True

def main(args=None):
    rclpy.init(args=args)
    node = RosieIONode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
