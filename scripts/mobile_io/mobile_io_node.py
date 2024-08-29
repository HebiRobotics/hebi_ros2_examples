#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import hebi
import os
import json
from time import sleep
from ament_index_python.packages import get_package_share_directory
from hebi_ros2_examples.hebi_util import create_mobile_io_from_config
from hebi_msgs.msg import MobileInput
from hebi_msgs.srv import SetLayoutJSON, SetLayoutFile  

class MobileIONode(Node):
    def __init__(self):
        super().__init__('mobile_io_node')

        # Declare and fetch only the config_package and config_file parameters
        self.declare_parameter('config_package', '')
        self.declare_parameter('config_file', '')

        self.config_package = self.get_parameter('config_package').get_parameter_value().string_value
        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value

        # Validate the configuration before proceeding
        if not all([self.config_package, self.config_file]):
            raise ValueError('Both config_package and config_file must be provided.')

        # Resolve the config file path using the package share directory
        package_share_directory = get_package_share_directory(self.config_package)
        self.config_file_path = os.path.join(package_share_directory, 'config', self.config_file)

        # Check if the config file path is valid
        if not os.path.isfile(self.config_file_path):
            raise FileNotFoundError(f"Config file not found: {self.config_file_path}")

        self.get_logger().debug(f'Config file path is valid: {self.config_file_path}')

        # Load the configuration
        self.get_logger().debug(f'Loading configuration from {self.config_file_path}')
        self.mobile_io_config = hebi.config.load_config(self.config_file_path)

        # Initialize the HEBI Lookup interface
        self.lookup = hebi.Lookup()
        sleep(2)  # Allow time for the lookup to discover devices

        # Initialize Mobile IO from config
        self.mobile_io = create_mobile_io_from_config(self.mobile_io_config, self.lookup, self)

        # Create a timer to run at 200 Hz
        self.timer = self.create_timer(1.0/200.0, self.timer_callback)

        # Create a publisher for /mobile_input
        qos_profile = QoSProfile(depth=10)
        self.input_publisher = self.create_publisher(MobileInput, 'mobile_input', qos_profile)

        # Initialize the input message
        self.max_buttons = 8
        self.max_axes = 8
        self.mobile_input_msg = MobileInput()
        self.mobile_input_msg.button_states = [0] * self.max_buttons
        self.mobile_input_msg.button_diffs = [0] * self.max_buttons
        self.mobile_input_msg.axis_states = [0.0] * self.max_axes

        # Create the services
        self.srv_set_layout_json = self.create_service(SetLayoutJSON, 'set_mobile_io_layout_json', self.set_layout_json_callback)
        self.srv_set_layout_file = self.create_service(SetLayoutFile, 'set_mobile_io_layout_file', self.set_layout_file_callback)

    def timer_callback(self):
        # This method will be called at 200 Hz
        self.get_logger().debug('Timer callback triggered at 200 Hz')

        # Update the mobile_io at each timer tick
        if self.mobile_io.update(timeout_ms=0):
            
            # Retrieve MobileIO state to populate input message
            self.populate_input_msg()

            # Publish input message
            self.input_publisher.publish(self.mobile_input_msg)

        # Additional logic for your arm or other components can go here
            
    def populate_input_msg(self):

        # Populate button fields
        for i in range(self.max_buttons):

            self.mobile_input_msg.button_states[i] = self.mobile_io.get_button_state(i+1)
            self.mobile_input_msg.button_diffs[i] = self.mobile_io.get_button_diff(i+1)

        # Populate axis fields
        for i in range(self.max_axes):

            self.mobile_input_msg.axis_states[i] = self.mobile_io.get_axis_state(i+1)

    def set_layout_json_callback(self, request, response):
        """
        Can be tested using:
        ros2 service call /set_mobile_io_layout_json hebi_msgs/srv/SetLayoutJSON "{layout_json: '[{\"id\": \"b1\", \"type\": \"button\", \"x\": -0.25, \"y\": -0.73, \"width\": 0.15, \"height\": 0.15, \"parameters\": {\"text\": \"❌/📈\", \"mode\": \"momentary\"}}]' }"
        """

        # Convert the raw JSON string to a JSON object
        try:
            layout_json = json.loads(request.layout_json)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON format: {e}')
            response.success = False
            return response

        # Send the layout JSON object to the mobile_io
        try:
            success = self.mobile_io.send_layout(layout=layout_json)
            if success:
                self.get_logger().info('Mobile IO layout updated successfully from JSON.')
                response.success = True
            else:
                self.get_logger().error('Failed to update Mobile IO layout from JSON.')
                response.success = False
        except Exception as e:
            self.get_logger().error(f'Failed to update Mobile IO layout from JSON: {e}')
            response.success = False

        return response

    def set_layout_file_callback(self, request, response):

        try:
            # Locate the layout file in the given package
            package_share_directory = get_package_share_directory(request.layout_package_name)
            layout_file_path = os.path.join(package_share_directory, 'config/layouts', request.layout_file_name)

            # Check if the layout file exists
            if not os.path.isfile(layout_file_path):
                raise FileNotFoundError(f"Layout file not found: {layout_file_path}")

            # Send the layout file path to the mobile_io
            success = self.mobile_io.send_layout(layout_file=layout_file_path)
            if success:
                self.get_logger().info('Mobile IO layout updated successfully from file.')
                response.success = True
            else:
                self.get_logger().error('Failed to update Mobile IO layout from file.')
                response.success = False

        except Exception as e:
            self.get_logger().error(f'Failed to update Mobile IO layout from file: {e}')
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MobileIONode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mobile IO Node is shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
