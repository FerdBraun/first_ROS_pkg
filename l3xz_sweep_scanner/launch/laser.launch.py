from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_sweep_scanner',
      executable='l3xz_sweep_scanner_node',
      name='sweep_node',
      namespace='l3xz',
      output='screen',
      parameters=[
          {'topic' : 'Laser'},
          {'serial_port' : '/dev/ttyUSB0'},
          {'rotation_speed': 2},
          {'sample_rate': 1000},
          {'frame_id' : 'laser_frame'}
      ]
    ),
    Node(package = "tf2_ros",
         executable = "static_transform_publisher",
         name="robot_link_to_laser_frame",
         namespace='l3xz',
         output='screen',
         arguments = ["0.06", "0", "0.2", "0", "0", "0", "robot_link", "laser_frame"]
    )
  ])
