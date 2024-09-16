#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SpiderRobotCommandSender(Node):
    def __init__(self):
        super().__init__('spider_robot_command_sender')
        self.publisher = self.create_publisher(String, '/spider_robot/command', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # Увеличим интервал до 2 секунд для наглядности
        self.command = "ВПЕРЕД"

    def timer_callback(self):
        msg = String()
        msg.data = self.command
        self.publisher.publish(msg)

        # Переключаем команду
        if self.command == "ВПЕРЕД":
            self.command = "НАЗАД"
        else:
            self.command = "ВПЕРЕД"

def main(args=None):
    rclpy.init(args=args)
    command_sender = SpiderRobotCommandSender()
    rclpy.spin(command_sender)
    command_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
