#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from time import sleep
import hebi
import numpy as np
import json
import requests


class HexapodIONode(Node):
    def __init__(self):
        super().__init__('HexapodIO_node')

        self.declare_parameter("family", "HEBI")
        self.declare_parameter("name", "mobileIO")

        self.mio = None
        self.timer_mio = self.create_timer(0.02, self.mio_callback)
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'velocity_command', 100)
        self.mode_sel_pub = self.create_publisher(Bool, 'mode_select', 100)

        self.xyz_scale = 0.175
        self.rot_scale = 0.4
        self.cmd_vel_msg = Twist()
        self.mode_sel_msg = Bool()

        if (not self.initialize()):
            self.get_logger().error("Could not initialize Mobile IO")
            return
            
    def mio_callback(self):
        self.mio.update()

        if self.mio.get_button_diff(4) == 1:
            self.mode_sel_msg.data = True
            self.mode_sel_pub.publish(self.mode_sel_msg)
        elif self.mio.get_button_diff(4) == -1:
            self.mode_sel_msg.data = False
            self.mode_sel_pub.publish(self.mode_sel_msg)

        if self.mio.get_button_state(8) == 1:
            # exit
            self.get_logger().info("Exiting...")
            self.mio.set_led_color('off')
            rclpy.shutdown()

        self.cmd_vel_msg.linear.x = self.mio.get_axis_state(8) * self.xyz_scale
        self.cmd_vel_msg.linear.y = -self.mio.get_axis_state(7) * self.xyz_scale
        self.cmd_vel_msg.linear.z = self.apply_deadzone(self.mio.get_axis_state(3), 0.25) * self.xyz_scale

        self.cmd_vel_msg.angular.y = self.mio.get_axis_state(2) * self.rot_scale
        self.cmd_vel_msg.angular.z = -self.mio.get_axis_state(1) * self.rot_scale

        # Don't publish if no joints are moving
        if self.cmd_vel_msg.linear.x == 0.0 and self.cmd_vel_msg.linear.y == 0.0 and self.cmd_vel_msg.linear.z == 0.0 and self.cmd_vel_msg.angular.y == 0.0 and self.cmd_vel_msg.angular.z == 0.0:
            return
        
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
    
    def apply_deadzone(self, raw, deadzone):
        if np.abs(raw) < deadzone:
            return 0.0
        if raw > 0.0:
            return (raw - deadzone) / (1.0 - deadzone)
        else:
            return (raw + deadzone) / (1.0 - deadzone)

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

        self.mio.set_snap(3, 0.0)
        self.mio.set_button_mode(4, 1)
        self.mio.set_button_output(4, 1)
        self.mio.set_button_output(8, 1)

        self.mio.set_led_color('green')

        return True

def main(args=None):
    rclpy.init(args=args)
    node = HexapodIONode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
