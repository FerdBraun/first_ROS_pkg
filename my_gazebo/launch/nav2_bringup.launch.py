from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'my_gazebo'
    
    # Путь к конфигурационному файлу NAV2
    nav2_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'description',
        'nav2_params.yaml'
    )
    
    # Путь к скрипту преобразования команд скорости
    cmd_vel_converter_script = os.path.join(
        get_package_share_directory(pkg_name),
        'topics',
        'cmd_vel_to_spider_commands.py'
    )

    cmd_vel_converter = Node(
        executable=cmd_vel_converter_script,
        name='cmd_vel_to_spider_commands',
        output='screen',
        parameters=[{'use_sim_time': True}],

    )

    # Узлы NAV2
    planner_script = os.path.join(
        get_package_share_directory(pkg_name),
        'topics',
        'planner.py'
    )

    planner= Node(
        executable=planner_script,
        name='cmd_vel_quantizer',
        output='screen',
        parameters=[{'use_sim_time': True}],

    )

    return LaunchDescription([
                              cmd_vel_converter,
                              planner
                              ] )