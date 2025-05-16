#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class TopicRelay(Node):
    def __init__(self):
        super().__init__('topic_relay')
        self.pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/map',
            self.callback,
            10)

    def callback(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TopicRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()