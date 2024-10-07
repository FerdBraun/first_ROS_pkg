import os

import launch_ros
from launch_ros.actions.node import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="kinect_ros2").find(
        "kinect_ros2"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/pointcloud.rviz")
    spider_robot_controllerV2_script = os.path.join(pkg_share, 'src/rotation.py')

    spider_robot_controllerV2 = Node(executable=spider_robot_controllerV2_script,
            name='rotation',
            parameters=[{'use_sim_time': True}] 
            )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            Node(
                package="kinect_ros2",
                executable="kinect_ros2_node",
                name="kinect_ros2",
                namespace="kinect",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
            ),
            Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='depth_to_world_transform',
            arguments=['0', '0', '0', '1.5708', '0', '-1.5708', 'world', 'kinect_depth']
            )

        ]
    )
