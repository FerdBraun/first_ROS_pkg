
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



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'INSECTOID'],
                    output='screen')
       


    spider_robot_controllerV2_script = os.path.join(get_package_share_directory(pkg_name), 'topics/spider_robot_comannd_centerV2.py')

    spider_robot_controllerV2 = Node(executable=spider_robot_controllerV2_script,
            name='spider_robot_comannd_centerV2',
            parameters=[{'use_sim_time': True}] )
    


    
    gzrm_script = os.path.join(get_package_share_directory(pkg_name), 'topics/gazebo_robot_movement.py')

    gzrm = Node(executable=gzrm_script,
            name='gzrm',
            parameters=[{'use_sim_time': True}] )




    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller"],
        parameters=[{'use_sim_time': True}] ,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}] ,
    )

    lidar_odom_sim_script = os.path.join(get_package_share_directory(pkg_name), 'topics/lazerOdometry_sim.py')
    lidar_odom_sim = Node(executable=lidar_odom_sim_script,
            name='lidar_odom_sim',
            parameters=[{'use_sim_time': True}] )
    
    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        #command_sender,
        #spider_robot_controller,
        spider_robot_controllerV2,
        joint_trajectory_controller_spawner,
        joint_state_broadcaster_spawner,
        lidar_odom_sim,
        #gzrm
    ])