#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateCombinerNode(Node):
    def __init__(self):
        super().__init__('joint_state_combiner_node')
        
        self.joint_state_base = JointState()
        self.joint_state_arm = JointState()

        # Initialize the subscribers for the joint states of the base and arm
        self.subscriber_base = self.create_subscription(JointState, 'omni_base/joint_states', self.base_state_callback, 10)
        self.subscriber_arm = self.create_subscription(JointState, 'arm/joint_states', self.arm_state_callback, 10)

        # Initialize the publisher for the combined joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Timer to publish the combined joint states
        self.timer = self.create_timer(0.005, self.publish_combined_joint_states)

        # Initialize the combined joint states message
        self.joint_state_combined = JointState()

    def base_state_callback(self, msg):
        # Store the base joint states
        self.joint_state_base = msg

    def arm_state_callback(self, msg):
        # Store the arm joint states
        self.joint_state_arm = msg
    
    def publish_combined_joint_states(self):
        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = self.joint_state_base.name + self.joint_state_arm.name
        joint_states.position = self.joint_state_base.position + self.joint_state_arm.position
        joint_states.velocity = self.joint_state_base.velocity + self.joint_state_arm.velocity
        joint_states.effort = self.joint_state_base.effort + self.joint_state_arm.effort
        self.publisher.publish(joint_states)

def main(args=None):
    rclpy.init(args=args)
    joint_state_combiner_node = JointStateCombinerNode()

    try:
        rclpy.spin(joint_state_combiner_node)
    except KeyboardInterrupt:
        pass

    joint_state_combiner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
