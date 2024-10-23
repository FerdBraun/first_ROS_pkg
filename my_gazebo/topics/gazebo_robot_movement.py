#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState, GetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
import math
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class ModelMover(Node):
    def __init__(self):
        super().__init__('model_mover')
        self.client = self.create_client(SetEntityState, '/demo/set_entity_state')
        self.get_state_client = self.create_client(GetEntityState, '/demo/get_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GetEntityState service not available, waiting...')

        self.position = None
        self.orientation = None

        self.subscription = self.create_subscription(
            String,
            'spider_robot/command',
            self.command_callback,
            10)

        self.future = None
        self.get_state_future = None

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

    def rotate_vector(self, vector, angle):
        rotation_matrix = [
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle), math.cos(angle)]
        ]
        return (
            rotation_matrix[0][0]*vector[0] + rotation_matrix[0][1]*vector[1],
            rotation_matrix[1][0]*vector[0] + rotation_matrix[1][1]*vector[1]
        )

    def get_current_state_callback(self, future):
        try:
            response = future.result()
            if response and response.success:
                self.position = response.state.pose.position
                self.orientation = response.state.pose.orientation
                self.process_command()
            else:
                self.get_logger().error('Failed to get current state')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def get_current_state(self):
        request = GetEntityState.Request()
        request.name = 'INSECTOID'
        self.get_state_future = self.get_state_client.call_async(request)
        self.get_state_future.add_done_callback(self.get_current_state_callback)

    def command_callback(self, msg):
        self.command = msg.data.upper()
        self.get_logger().info(f'Received command: {self.command}')

        if self.position is None or self.orientation is None:
            self.get_current_state()
        else:
            self.process_command()

    def process_command(self):
        # Получаем текущие углы Эйлера из кватерниона
        roll, pitch, yaw = euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])

        # Обновление позиции и ориентации относительно осей модели
        if self.command == 'F':
            forward_vector = self.rotate_vector((0, 1), yaw)
            self.position.x += forward_vector[0] * 0.1
            self.position.y += forward_vector[1] * 0.1
        elif self.command == 'B':
            backward_vector = self.rotate_vector((0, -1), yaw)
            self.position.x += backward_vector[0] * 0.1
            self.position.y += backward_vector[1] * 0.1
        elif self.command == 'L':
            yaw -= math.radians(10)
        elif self.command == 'R':
            yaw += math.radians(10)
        else:
            self.get_logger().warn(f'Unknown command: {self.command}')
            return

        # Обновляем кватернион на основе нового угла поворота
        x, y, z, w = quaternion_from_euler(0, 0, yaw)
        self.orientation.x = x
        self.orientation.y = y
        self.orientation.z = z
        self.orientation.w = w

        # Добавляем статическое изменение координаты Z на 0.1
        self.position.z = 0.2

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
