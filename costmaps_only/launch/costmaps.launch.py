from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('costmaps_only'),
        'config'
    )

    params_file = os.path.join(config_dir, 'nav2_params.yaml')

    return LaunchDescription([
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmaps',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['costmaps_server']}]
        ),
        Node(
            package='costmaps_only',
            executable='costmaps_server_node',
            name='costmaps_server',
            output='screen',
            parameters=[params_file]
        ),
        
    ])