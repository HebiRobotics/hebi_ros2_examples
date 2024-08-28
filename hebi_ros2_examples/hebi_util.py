import hebi
import os
from hebi.util import create_mobile_io
from time import sleep

def create_mobile_io_from_config(lookup, config, config_file, node):

    cfg_dir = os.path.dirname(os.path.abspath(config_file))  # If mobile_io becomes a unique field in config, this will no longer be required

    # Load Mobile IO config as a dictionary
    mobile_io_dict = {}
    required_keys = {'mobile_io_family', 'mobile_io_name', 'mobile_io_layout'}

    # Validate the mobile_io configuration
    if set(required_keys).issubset(config.user_data.keys()):

        # Check that all required fields are present and are non-empty strings
        if all(isinstance(config.user_data[key], str) for key in required_keys):

            mobile_io_dict['family'] = config.user_data.get('mobile_io_family')
            mobile_io_dict['name'] = config.user_data.get('mobile_io_name')
            mobile_io_dict['layout'] = os.path.join(cfg_dir, config.user_data.get('mobile_io_layout'))
        else:
            raise TypeError(f'HEBI config "user_data"\'s {required_keys} fields must contain strings, not parseable as strings')
    else:
        raise ValueError(f'HEBI config "user_data" field must contain the keys: {required_keys}')

    # Set up Mobile IO from config
    mobile_io = None
    node.get_logger().info('Waiting for Mobile IO device to come online...')

    while mobile_io is None:
        mobile_io = create_mobile_io(lookup, mobile_io_dict['family'], mobile_io_dict['name'])

        if mobile_io is None:
            node.get_logger().warn(f"Couldn't find Mobile IO. Check name, family, or device status...")
            sleep(1)

    node.get_logger().info('Mobile IO device found. Sending layout...')
    mobile_io.send_layout(layout_file=mobile_io_dict['layout'])
    mobile_io.update()
    node.get_logger().info('Mobile IO successfully initialized.')

    return mobile_io

def create_gripper_from_config(lookup, config, config_file, arm):

    cfg_dir = os.path.dirname(os.path.abspath(config_file)) # If gripper becomes a unique field in config, this will no longer be required

    # Load gripper config as a dictionary
    gripper_dict = {}
    required_keys = {'gripper_family', 'gripper_name', 'gripper_gains', 'gripper_close_effort', 'gripper_open_effort'}
    required_keys_str = {'gripper_family', 'gripper_name', 'gripper_gains'}
    required_keys_float = {'gripper_close_effort', 'gripper_open_effort'}
        
    # Validate the mobile_io configuration
    if set(required_keys).issubset(config.user_data.keys()):

        # Check that all required fields are present and are non-empty strings
        if (all(isinstance(config.user_data[key], str) for key in required_keys_str) and
            all(isinstance(config.user_data[key], float) for key in required_keys_float)):

            gripper_dict['family'] = config.user_data.get('gripper_family')
            gripper_dict['name'] = config.user_data.get('gripper_name')
            gripper_dict['gains'] = os.path.join(cfg_dir, config.user_data.get('gripper_gains'))
            gripper_dict['close_effort'] = config.user_data.get('gripper_close_effort')
            gripper_dict['open_effort'] = config.user_data.get('gripper_open_effort')
        else:
            raise TypeError(f'HEBI config "user_data"\'s {required_keys} fields must contain strings, not parseable as strings')
    else:
        raise ValueError(f'HEBI config "user_data" field must contain the keys: {required_keys}')

    # Add the gripper
    gripper = None
    if any(gripper_dict):
        group = lookup.get_group_from_names([gripper_dict.get('family')], [gripper_dict.get('name')])
        if group is None:
            raise RuntimeError("Incorrect family or name of gripper")

        gripper = hebi.arm.Gripper(group, gripper_dict.get('close_effort'), gripper_dict.get('open_effort'))
        gripper.load_gains(gripper_dict.get('gains'))
        if not arm is None:
            arm.set_end_effector(gripper)
        
    return gripper