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
        self.step_count = 0  # Счетчик шагов

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [
            'RL-1-1', 'RL-1-2', 'RL-1-3',           #0 1 2
            'RL-2-1', 'RL-2-2', 'RL-2-3',           #3 4 5
            'RL-3-1', 'RL-3-2', 'RL-3-3',           #6 7 8

            'LL-1-1', 'LL-1-2', 'LL-1-3',           #9 10 11
            'LL-2-1', 'LL-2-2', 'LL-2-3',           #12 13 14
            'LL-3-1', 'LL-3-2', 'LL-3-3'            #15 16 17
        ]

        # Определяем позиции для каждой ноги в зависимости от шага
        positions = self.get_leg_positions(self.step_count)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=1)
        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f'Published trajectory with step count: {self.step_count}')

        # Увеличиваем счетчик шагов
        self.step_count += 1
        if self.step_count >= 3:  # Возвращаемся к началу цикла после 6 шагов
            self.step_count = 0

    def get_leg_positions(self, step_count):
        # Определяем позиции для каждой ноги в зависимости от шага
        positions = [0.0, 0.0, 0.0] * 6  # Исходное положение

        if step_count == 0:
            positions[12:15] = [-0.3, -0.8, -0.8] 
            positions[0:3] = [0.3, 0.8, 0.8] 
            positions[6:9] = [0.3, 0.8, 0.8] 


            positions[9:12] = [0.5, -0.0, -0.0] 
            #positions[3:6] = [-0.5, 0.0, 0.0] 
            positions[15:18] = [0.5, -0.0, -0.0] 
        #     positions[0:3] = [0.3, 0.4, 0.4]  # Поднимаем первую ногу
        elif step_count == 1:
            positions[9:12] = [-0.3, -0.4, -0.4] 
            positions[3:6] = [0.3, 0.4, 0.4] 
            positions[15:18] = [-0.3, -0.4, -0.4]    


            positions[12:15] = [0.0, 0.0, 0.0] 
            positions[0:3] = [0.0, 0.0, 0.0] 
            positions[6:9] = [0.0, 0.0, 0.0]   # Поднимаем вторую ногу
        # elif step_count == 2:
        #     positions[3:6] = [0.2, 0.3, 0.3]  # Опускаем вторую ногу
        #     positions[6:9] = [0.1, 0.4, 0.4]  # Поднимаем третью ногу
        # elif step_count == 3:
        #     positions[6:9] = [0.2, 0.3, 0.3]  # Опускаем третью ногу
        #     positions[9:12] = [0.1, -0.4, -0.4]  # Поднимаем четвертую ногу
        # elif step_count == 4:
        #     positions[9:12] = [0.2, -0.3, -0.3]  # Опускаем четвертую ногу
        #     positions[12:15] = [0.1, -0.4, -0.4]  # Поднимаем пятую ногу
        # elif step_count == 5:
        #     positions[12:15] = [0.2, -0.3, -0.3]  # Опускаем пятую ногу
        #     positions[15:18] = [0.1, -0.4, -0.4]  # Поднимаем шестую ногу

        return positions

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
