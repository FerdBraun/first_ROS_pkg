#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetModelState, SetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
import math
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class ModelMover(Node):
    def __init__(self):
        super().__init__('model_mover')
        self.client_set_state = self.create_client(SetEntityState, '/demo/set_entity_state')
        self.client_get_state = self.create_client(GetModelState, '/get_model_state')
        
        while not (self.client_set_state.wait_for_service(timeout_sec=1.0) and 
                  self.client_get_state.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('Services not available, waiting...')
        
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

    def rotate_vector(self, vector, angle):
        rotation_matrix = [
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle), math.cos(angle)]
        ]
        return (
            rotation_matrix[0][0]*vector[0] + rotation_matrix[0][1]*vector[1],
            rotation_matrix[1][0]*vector[0] + rotation_matrix[1][1]*vector[1]
        )

    def get_current_position(self):
        request = GetModelState.Request()
        request.model_name = 'INSECTOID'
        future = self.client_get_state.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().pose

    def command_callback(self, msg):
        command = msg.data.upper()
        self.get_logger().info(f'Received command: {command}')

        # Получаем текущее положение модели
        current_pose = self.get_current_position()

        # Получаем текущие углы Эйлера из кватерниона
        roll, pitch, yaw = euler_from_quaternion([
            current_pose.orientation.x, current_pose.orientation.y,
            current_pose.orientation.z, current_pose.orientation.w
        ])

        # Обновление позиции и ориентации относительно осей модели
        if command == 'F':
            forward_vector = self.rotate_vector((0, 1), yaw)
            current_pose.position.x += forward_vector[0] * 0.1
            current_pose.position.y += forward_vector[1] * 0.1
        elif command == 'B':
            backward_vector = self.rotate_vector((0, -1), yaw)
            current_pose.position.x += backward_vector[0] * 0.1
            current_pose.position.y += backward_vector[1] * 0.1
        elif command == 'L':
            roll -= math.radians(10)
        elif command == 'R':
            roll += math.radians(10)
        elif command == 'U':
            pitch += math.radians(10)
        elif command == 'D':
            pitch -= math.radians(10)
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return

        # Обновляем кватернион на основе новых углов поворота
        x, y, z, w = quaternion_from_euler(roll, pitch, yaw)
        current_pose.orientation.x = x
        current_pose.orientation.y = y
        current_pose.orientation.z = z
        current_pose.orientation.w = w

        # Добавляем статическое изменение координаты Z на 0.1
        current_pose.position.z = 0.1

        # Отправка запроса на перемещение модели
        request = SetEntityState.Request()
        request.state.name = 'INSECTOID'
        request.state.pose = current_pose
        request.state.twist.linear = Twist().linear
        request.state.twist.angular = Twist().angular
        
        self.future = self.client_set_state.call_async(request)
        self.get_logger().info(f'Sent request to move model to position ({current_pose.position.x}, {current_pose.position.y}, {current_pose.position.z})')

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
