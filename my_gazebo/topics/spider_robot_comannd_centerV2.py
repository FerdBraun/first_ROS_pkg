#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # Увеличим интервал до 2 секунд для наглядности
        self.direction = 1  # Направление движения: 1 - вперед, -1 - назад

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [
            'RL-1-1', 'RL-1-2', 'RL-1-3',
            'RL-2-1', 'RL-2-2', 'RL-2-3',
            'RL-3-1', 'RL-3-2', 'RL-3-3',

            'LL-1-1', 'LL-1-2', 'LL-1-3',
            'LL-2-1', 'LL-2-2', 'LL-2-3',
            'LL-3-1', 'LL-3-2', 'LL-3-3'
        ]

        point = JointTrajectoryPoint()
        point.positions = [
            0.0 * self.direction, 0.3 * self.direction, 0.3 * self.direction,
            0.0 * self.direction, 0.3 * self.direction, 0.3* self.direction,
            0.0 * self.direction, 0.3 * self.direction, 0.3 * self.direction,

            0.0 * self.direction, -0.3 * self.direction, -0.3 * self.direction,
            0.0 * self.direction, -0.3 * self.direction, -0.3 * self.direction,
            0.0 * self.direction, -0.3 * self.direction, -0.3* self.direction
        ]
        point.time_from_start = Duration(sec=1)
        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f'Published trajectory with direction: {self.direction}')

        # Переключаем направление
        self.direction *= -1

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
