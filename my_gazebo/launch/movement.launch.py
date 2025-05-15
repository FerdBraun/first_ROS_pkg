
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


    gzrm_script = os.path.join(get_package_share_directory(pkg_name), 'topics/gazebo_robot_movement.py')

    gzrm = Node(executable=gzrm_script,
            name='gzrm',
            parameters=[{'use_sim_time': True}] )

    command_sender_script = os.path.join(get_package_share_directory(pkg_name), 'topics/spider_robot_command_sender.py')

    command_sender = Node(executable=command_sender_script,
            name='command_sender',
            parameters=[{'use_sim_time': True}] )
    
    socket_sender_script = os.path.join(get_package_share_directory(pkg_name), 'topics/socket_transporter.py')

    socket_sender = Node(executable=socket_sender_script,
            name='socket_sender',
            parameters=[{'use_sim_time': True}] )

    # Run the node
    return LaunchDescription([
        #gzrm,
        command_sender,
        socket_sender
    ])