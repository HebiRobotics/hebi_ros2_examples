#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import hebi
import os
from hebi.util import create_mobile_io
from time import sleep
from ament_index_python.packages import get_package_share_directory

class MobileIONode(Node):
    def __init__(self):
        super().__init__('mobile_io_node')

        # Declare and fetch only the relevant parameters
        self.declare_parameter('user_data.mobile_io_family', '')
        self.declare_parameter('user_data.mobile_io_name', '')
        self.declare_parameter('user_data.mobile_io_layout', '')
        self.declare_parameter('config_package', '')

        self.mobile_io_family = self.get_parameter('user_data.mobile_io_family').get_parameter_value().string_value
        self.mobile_io_name = self.get_parameter('user_data.mobile_io_name').get_parameter_value().string_value
        self.mobile_io_layout = self.get_parameter('user_data.mobile_io_layout').get_parameter_value().string_value
        self.config_package = self.get_parameter('config_package').get_parameter_value().string_value

        # Log the parameters to confirm they were parsed correctly
        self.get_logger().debug(f'Parameter user_data.mobile_io_family: {self.mobile_io_family}')
        self.get_logger().debug(f'Parameter user_data.mobile_io_name: {self.mobile_io_name}')
        self.get_logger().debug(f'Parameter user_data.mobile_io_layout: {self.mobile_io_layout}')
        self.get_logger().debug(f'Parameter config_package: {self.config_package}')

        # Validate the mobile_io configuration before constructing paths
        if not all([self.mobile_io_family, self.mobile_io_name, self.mobile_io_layout, self.config_package]):
            raise ValueError('Mobile IO configuration must contain non-empty strings for family, name, layout, and config_package.')

        # Resolve the layout file path using the package share directory
        package_share_directory = get_package_share_directory(self.config_package)
        self.layout_path = os.path.join(package_share_directory, 'config', self.mobile_io_layout)

        # Check if the layout path is valid
        if not os.path.isfile(self.layout_path):
            raise FileNotFoundError(f"Layout file not found: {self.layout_path}")

        self.get_logger().debug(f'Layout file path is valid: {self.layout_path}')

        # Initialize Mobile IO
        self.mobile_io = self.create_mobile_io_from_config()

    def create_mobile_io_from_config(self):
        mobile_io_dict = {
            'family': self.mobile_io_family,
            'name': self.mobile_io_name,
            'layout': self.layout_path
        }

        # Set up Mobile IO from config
        self.get_logger().info('Waiting for Mobile IO device to come online...')
        num_retries = 10

        for i in range(num_retries):
            mobile_io = create_mobile_io(hebi.Lookup(), mobile_io_dict['family'], mobile_io_dict['name'])

            if mobile_io is None:
                self.get_logger().warn(f"Couldn't find Mobile IO. Check name, family, or device status... Retry {i+1}/{num_retries}")
                sleep(1)
                if i == num_retries - 1:
                    raise RuntimeError("Failed to create Mobile IO from config.")
            else:
                break

        mobile_io.send_layout(layout_file=mobile_io_dict['layout'])
        mobile_io.update()
        self.get_logger().info('Mobile IO successfully initialized.')

        return mobile_io

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
