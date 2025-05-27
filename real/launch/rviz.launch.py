from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ïîëó÷àåì ïóòü ê ïàêåòó
    package_name = 'real'
    package_share_directory = get_package_share_directory(package_name)

    # Óêàçûâàåì ïóòü ê êîíôèãóðàöèîííîìó ôàéëó RViz
    rviz_config_path = os.path.join(package_share_directory, 'rviz_config', 'rviz_view.rviz')

    # Ñîçäàåì îïèñàíèå çàïóñêà
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
