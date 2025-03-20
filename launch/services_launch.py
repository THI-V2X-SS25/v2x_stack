import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():        
    config_file = os.path.join(
        get_package_share_directory('v2x_stack_btp'), 'config', 'services_params.yml'
    )  
    
    with open(config_file, 'r') as file:
        config_data = yaml.safe_load(file)
    
    services = config_data['services']
    
    nodes_to_launch = []

    # Iterate over the services dictionary
    for service_name, service_config in services.items():        
        if service_config['enabled']:            
            node = launch_ros.actions.Node(
                package=service_config['package'],
                executable=service_config['executable'],
                name=service_config['name']
            )
            nodes_to_launch.append(node)
    
    return launch.LaunchDescription(nodes_to_launch)

    