from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
import json
def generate_launch_description():
    map_yaml_path = '/home/zega/Desktop/ros_dev/src/costmaps_only/map.yaml'
    # Основная нода map_server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': map_yaml_path
        }]
    )
   
  
    # Команды с задержками
    configure_cmd = ExecuteProcess(
        cmd=[FindExecutable(name='ros2'), 
             'lifecycle', 'set', '/map_server', 'configure'],
        shell=True
    )

    activate_cmd = ExecuteProcess(
        cmd=[FindExecutable(name='ros2'),
             'lifecycle', 'set', '/map_server', 'activate'],
        shell=True
    )

    
    load_map_cmd = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'service', 'call',
            '/map_server/load_map',
            'nav2_msgs/srv/LoadMap',
            '{map_url: "' + map_yaml_path + '"}'
        ],
        output='screen'
    )

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map']
        ),
        map_server_node,
        # Конфигурация через 3 сек после старта
        TimerAction(
            period=3.0,
            actions=[configure_cmd]
        ),
        # Активация через 5 сек после конфигурации
        TimerAction(
            period=5.0,
            actions=[activate_cmd]
        ),
        # Загрузка карты через 2 сек после активации
        TimerAction(
            period=10.0,
            actions=[load_map_cmd]
        )
    ])
