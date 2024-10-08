#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.srv import ApplyBodyWrench

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench

class ForceApplier(Node):
    def __init__(self):
        super().__init__('force_applier')
        self.client = self.create_client(ApplyBodyWrench, '/gazebo/apply_body_wrench')
        self.get_logger().info('Waiting for service /gazebo/apply_body_wrench...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def apply_force(self):
        # Create the request for the ApplyBodyWrench service
        request = ApplyBodyWrench.Request()
        request.body_name = 'base_link'
        request.reference_frame = 'base_link'  # Assuming the force is applied to the base link
        request.wrench.force.x = 0.0
        request.wrench.force.y = 0.0
        request.wrench.force.z = 0.0
        request.wrench.torque.x = 0.0
        request.wrench.torque.y = 0.0
        request.wrench.torque.z = 0.0
        request.start_time.sec = 0
        request.start_time.nanosec = 0
        request.duration.sec = 10
        request.duration.nanosec = 0

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Force applied successfully')
        else:
            self.get_logger().error('Failed to apply force')

def main(args=None):
    rclpy.init(args=args)
    force_applier = ForceApplier()
    rclpy.spin(force_applier)
    force_applier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
