#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def cmd_vel_callback(self, data):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Извлекаем скорость и угловую скорость из сообщения Twist
        vx = data.linear.x
        vy = data.linear.y
        vth = data.angular.z

        # Применяем кинематическую модель движения
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Создаем сообщение Odometry и публикуем его
        self.publish_odometry(current_time, vx, vy, vth)

        self.last_time = current_time

    def timer_callback(self):
        current_time = self.get_clock().now()

        # Создаем сообщение Odometry и публикуем его
        self.publish_odometry(current_time, 0.0, 0.0, 0.0)

    def publish_odometry(self, current_time, vx, vy, vth):
        # Создаем сообщение Odometry и публикуем его
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.th / 2)
        odom.pose.pose.orientation.w = math.cos(self.th / 2)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

        # Создаем сообщение TransformStamped и публикуем преобразование координат
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.th / 2)
        t.transform.rotation.w = math.cos(self.th / 2)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdometryNode()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# # Двигаться вперед на 10 см
# ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {y: -0.1}, angular: {z: 0.0}}"

# # Двигаться назад на 10 см
# ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: -0.1}, angular: {z: 0.0}}"

# # Повернуться налево на 90 градусов
# ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.523599}}"

# # Повернуться направо на 90 градусов
# ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: -0.523599}}"
