#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
import math

class ModelMover(Node):
    def __init__(self):
        super().__init__('model_mover')
        self.client = self.create_client(SetEntityState, '/demo/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.position = Point(x=0.0, y=0.0, z=0.0)
        self.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        
        self.subscription = self.create_subscription(
            String,
            'spider_robot/command',
            self.command_callback,
            10)
        
        self.future = None

    def move_model(self):
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                if result is not None:
                    self.get_logger().info('Model moved successfully')
                else:
                    self.get_logger().error('Failed to move model')
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
            finally:
                self.future = None

    def command_callback(self, msg):
        command = msg.data.upper()
        self.get_logger().info(f'Received command: {command}')

        # Обновление позиции и ориентации
        if command == 'F':
            self.position.y -= 0.1
        elif command == 'B':
            self.position.y += 0.1
        elif command == 'L':
            current_yaw = 2 * math.acos(self.orientation.w)
            new_yaw = current_yaw - math.radians(10)
            self.orientation.w = math.cos(new_yaw / 2)
            self.orientation.z = math.sin(new_yaw / 2)
        elif command == 'R':
            current_yaw = 2 * math.acos(self.orientation.w)
            new_yaw = current_yaw + math.radians(10)
            self.orientation.w = math.cos(new_yaw / 2)
            self.orientation.z = math.sin(new_yaw / 2)
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return

        # Добавляем статическое изменение координаты Z на 0.1
        self.position.z = 0.1

        # Отправка запроса на перемещение модели
        request = SetEntityState.Request()
        request.state.name = 'INSECTOID'
        request.state.pose = Pose(position=self.position, orientation=self.orientation)
        request.state.twist.linear = Twist().linear
        request.state.twist.angular = Twist().angular
        
        self.future = self.client.call_async(request)
        self.get_logger().info(f'Sent request to move model to position ({self.position.x}, {self.position.y}, {self.position.z})')

def main(args=None):
    rclpy.init(args=args)
    model_mover = ModelMover()
    
    try:
        while True:
            rclpy.spin_once(model_mover)
            model_mover.move_model()
    except KeyboardInterrupt:
        model_mover.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        model_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
