#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Twist, Vector3
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.subscription = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.imu_callback,
            10
        )
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        self.last_time = self.get_clock().now()

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Update position and orientation
        self.x += msg.linear_acceleration.x * dt
        self.y += msg.linear_acceleration.y * dt

        # Update yaw using angular velocity around Z axis
        self.yaw += msg.angular_velocity.z * dt

        # Publish odometry message
        self.publish_odometry(msg.orientation, current_time, msg)

        self.last_time = current_time

    def publish_odometry(self, orientation, current_time, msg):
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation = orientation

        odom.twist.twist.linear.x = msg.linear_acceleration.x
        odom.twist.twist.linear.y = msg.linear_acceleration.y
        odom.twist.twist.linear.z = 0.0  # Ignore Z acceleration
        odom.twist.twist.angular.x = 0.0  # Ignore angular velocity around X axis
        odom.twist.twist.angular.y = 0.0  # Ignore angular velocity around Y axis
        odom.twist.twist.angular.z = msg.angular_velocity.z

        self.publisher_.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation = orientation

        self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
