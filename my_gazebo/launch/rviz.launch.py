from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Получаем путь к пакету
    package_name = 'my_gazebo'
    package_share_directory = get_package_share_directory(package_name)

    # Указываем путь к конфигурационному файлу RViz
    rviz_config_path = os.path.join(package_share_directory, 'rviz_config', 'rviz_view.rviz')

    # Создаем описание запуска
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
