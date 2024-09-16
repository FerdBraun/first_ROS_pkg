#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

class SpiderRobotController(Node):
    def __init__(self):
        super().__init__('spider_robot_controller')

        # Подписка на топик для получения команд высокого уровня
        self.subscription = self.create_subscription(
            String,
            '/spider_robot/command',
            self.command_callback,
            10
        )

        # Паблишеры для отправки команд на приводы
        self.joint1_pub = self.create_publisher(Float64, '/spider_robot/joint1_cmd', 10)
        self.joint2_pub = self.create_publisher(Float64, '/spider_robot/joint2_cmd', 10)
        # Добавьте паблишеры для других суставов

    def command_callback(self, msg):
        command = msg.data
        if command == "ВПЕРЕД":
            self.move_forward()
        elif command == "НАЗАД":
            self.move_backward()
        # Добавьте обработку других команд

    def move_forward(self):
        # Пример движения вперед
        self.publish_joint_command(self.joint1_pub, 0.5)
        self.publish_joint_command(self.joint2_pub, -0.5)
        # Добавьте команды для других суставов

    def move_backward(self):
        # Пример движения назад
        self.publish_joint_command(self.joint1_pub, -0.5)
        self.publish_joint_command(self.joint2_pub, 0.5)
        # Добавьте команды для других суставов

    def publish_joint_command(self, publisher, value):
        msg = Float64()
        msg.data = value
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = SpiderRobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
