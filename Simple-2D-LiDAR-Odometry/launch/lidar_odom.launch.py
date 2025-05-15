
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    Node(
            package='lidar_odometry',  # Замените на имя вашего пакета
            executable='lidar_odometry_node',  # Замените на имя вашего исполняемого файла
            name='lidar_odometry_node',
            output='screen',
            parameters=[
                {'max_correspondence_distance': 0.5},
                {'transformation_epsilon': 0.005},
                {'maximum_iterations': 600.0},
                {'scan_topic_name': '/scan_filtered'},
                {'odom_topic_name': 'odom'},
                {'use_sim_time': True}
            ]
        )
    ])
