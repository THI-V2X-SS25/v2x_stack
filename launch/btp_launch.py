import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('v2x_stack_btp'), 'config', 'udp_params.yaml'
    )
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="v2x_stack_btp",
            executable="v2x_stack_btp_udp_dispacher_node",
            name="udp_dispatcher"
        ),
        launch_ros.actions.Node(
            package="v2x_stack_btp",
            executable="v2x_stack_btp_btp_data_node",
            name="btp_data"
        )
    ])
