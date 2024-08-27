#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import hebi
import os
import yaml
from time import sleep
from ament_index_python.packages import get_package_share_directory
from hebi_ros2_examples.hebi_util import create_mobile_io_from_config

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
        self.get_logger().info(f'Loading configuration from {self.config_file_path}')
        self.example_config = hebi.config.load_config(self.config_file_path)

        # Initialize the HEBI Lookup interface
        self.lookup = hebi.Lookup()
        sleep(2)  # Allow time for the lookup to discover devices

        # Initialize Mobile IO from config
        self.mobile_io = create_mobile_io_from_config(self.lookup, self.example_config, self.config_file_path)

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
