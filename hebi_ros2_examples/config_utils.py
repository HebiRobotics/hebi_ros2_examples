import tempfile
import yaml

def convert_to_ros_params(context, config_file_path, node_name):
    # Perform substitution in the context of launch
    config_file_path = config_file_path.perform(context)
    
    with open(config_file_path, 'r') as file:
        config_data = yaml.safe_load(file)
    
    ros_params = {node_name: {'ros__parameters': config_data}}
    
    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.yaml')
    with open(temp_file.name, 'w') as file:
        yaml.dump(ros_params, file)
    
    return temp_file.name
