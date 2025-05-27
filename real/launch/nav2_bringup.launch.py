from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    pkg_name = 'real'
    
    nav2_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('costmaps_only'),
            'launch',
            'costmaps.launch.py'
        ]),
    )
    # Путь к скрипту преобразования команд скорости
    controller_script = os.path.join(
        get_package_share_directory(pkg_name),
        'topics',
        'controller.py'
    )
    mapnav2_script = os.path.join(
        get_package_share_directory(pkg_name),
        'topics',
        'nav2MapTranslator.py'
    )
    mapnav2 = Node(
        executable=mapnav2_script,
        name='mapnav2',
        output='screen',
        parameters=[{'use_sim_time': False}],

    )
    controller = Node(
        executable=controller_script,
        name='controller',
        output='screen',
        parameters=[{'use_sim_time': False}],

    )

    # Узлы NAV2
    planner_script = os.path.join(
        get_package_share_directory(pkg_name),
        'topics',
        'planner.py'
    )

    planner= Node(
        executable=planner_script,
        name='planner',
        output='screen',
        parameters=[{'use_sim_time': False}],

    )

    return LaunchDescription([
                                mapnav2,
                                nav2_launch,
                                controller,
                                planner
                              ] )
