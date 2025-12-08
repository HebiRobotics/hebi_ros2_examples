#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from hebi_msgs.msg import TreadedBaseState, TreadyFlipperVelocityCommand
from std_srvs.srv import SetBool
from time import sleep
import hebi
import numpy as np

# MobileIO Button Config
button_config = {
'reset_pose_btn': 1,
'joined_flipper_btn': 6,
'quit_btn': 8,
'slider_flip1': 3,
'slider_flip2': 4,
'slider_flip3': 5,
'slider_flip4': 6,
'joy_fwd': 2,
'joy_rot': 1
}


class TreadyIONode(Node):

    FLIPPER_VEL_SCALE = 1  # rad/sec

    def __init__(self):
        super().__init__('tready_io_node')

        self.declare_parameter("family", "Chevron")
        self.declare_parameter("name", "mobileIO")

        self.SPEED_MAX_LIN = 0.15  # m/s
        self.SPEED_MAX_ROT = self.SPEED_MAX_LIN / 0.4 # rad/s

        self.mio = None
        self.timer_mio = self.create_timer(0.02, self.parse_mobile_io_feedback)
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.flipper_vel_pub = self.create_publisher(TreadyFlipperVelocityCommand, 'flipper_vel', 10)
        
        self.treaded_base_state_sub = self.create_subscription(
            TreadedBaseState,
            'state',
            self.treaded_base_state_callback,
            10)
        self.state_msg = TreadedBaseState()

        # Service client 
        self.home_flippers_service = self.create_client(SetBool, 'home_flippers')
        self.align_flippers_service = self.create_client(SetBool, 'align_flippers')

        self.should_reset = False
        self.align_flippers = False
        self.joy_vel_fwd = 0.0
        self.joy_vel_rot = 0.0
        self.flipper_vels = np.zeros(4)

        if (not self.initialize()):
            self.get_logger().error("Could not initialize Mobile IO")
            return
    
    def treaded_base_state_callback(self, msg):
        self.state_msg = msg
    
    def parse_mobile_io_feedback(self):
        global button_config
        
        self.mio.update()        

        self.should_reset = bool(self.mio.get_button_state(button_config['reset_pose_btn']))

        flip1 = self.mio.get_axis_state(button_config['slider_flip1'])
        flip2 = self.mio.get_axis_state(button_config['slider_flip2'])
        flip3 = self.mio.get_axis_state(button_config['slider_flip3'])
        flip4 = self.mio.get_axis_state(button_config['slider_flip4'])
        f_vel1 = flip1 * self.FLIPPER_VEL_SCALE
        f_vel2 = -1 * flip2 * self.FLIPPER_VEL_SCALE
        f_vel3 = flip3 * self.FLIPPER_VEL_SCALE
        f_vel4 = -1 * flip4 * self.FLIPPER_VEL_SCALE
        
        self.flipper_vels = [f_vel1, f_vel2, f_vel3, f_vel4]

        # Chassis Control
        self.joy_vel_fwd = self.mio.get_axis_state(button_config['joy_fwd']) * self.SPEED_MAX_LIN
        self.joy_vel_rot = self.mio.get_axis_state(button_config['joy_rot']) * self.SPEED_MAX_ROT

        # Publish
        self.cmd_vel_callback()
        self.flipper_vel_callback()

    def cmd_vel_callback(self):
        msg = Twist()
        msg.linear.x = self.joy_vel_fwd
        msg.linear.y = 0.0
        msg.angular.z = self.joy_vel_rot

        self.cmd_vel_pub.publish(msg)
    
    def flipper_vel_callback(self):
        msg = TreadyFlipperVelocityCommand()
        msg.front_left = self.flipper_vels[0]
        msg.front_right = self.flipper_vels[1]
        msg.back_left = self.flipper_vels[2]
        msg.back_right = self.flipper_vels[3]

        self.flipper_vel_pub.publish(msg)

    def initialize(self):
        global button_config

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

        # set mobileIO control config
        self.mio.set_led_color("blue")
        self.mio.set_snap(button_config['slider_flip1'], 0.0)
        self.mio.set_snap(button_config['slider_flip2'], 0.0)
        self.mio.set_snap(button_config['slider_flip3'], 0.0)
        self.mio.set_snap(button_config['slider_flip4'], 0.0)

        self.mio.set_button_mode(button_config['joined_flipper_btn'], 1)

        self.mio.set_button_output(button_config['joined_flipper_btn'], 0)
        self.mio.set_button_output(button_config['quit_btn'], 0)

        return True

def main(args=None):
    rclpy.init(args=args)
    node = TreadyIONode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
