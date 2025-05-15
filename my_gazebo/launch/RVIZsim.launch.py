import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'my_gazebo'
    file_subpath = 'description/example_robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    bridge_script = os.path.join(get_package_share_directory(pkg_name), 'topics/bridge_client.py')

    bridge = Node(executable=bridge_script,
            name='spider_robot_comannd_centerV2',
            parameters=[{'use_sim_time': True}] )

    # Run the node
    return LaunchDescription([
        bridge,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='depth_to_world_transform',
            parameters=[{'use_sim_time': True}],
            arguments=['0', '0', '0.8', '0', '0', '-1.5708', 'base_link', 'kinect_depth']
            )


    ])