#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
from inputs import get_gamepad

class GamepadController:
    def __init__(self):
        self.node = rclpy.create_node('gamepad_publisher')
        self.publisher = self.node.create_publisher(String, '/spider_robot/command', 10)
        self.last_command = None
    
    def process_gamepad_events(self):
        while rclpy.ok():
            events = get_gamepad()
            for event in events:
                self.handle_event(event)
    
    def handle_event(self, event):
        msg = String()
        
        # Обработка кнопок (пример для XInput геймпада)
        if event.ev_type == 'Key':
            if event.code == 'BTN_SOUTH' and event.state == 1:  # Кнопка A (для движения вперед)
                msg.data = 'F'
            elif event.code == 'BTN_WEST' and event.state == 1:  # Кнопка X (для движения назад)
                msg.data = 'B'
            elif event.code == 'BTN_NORTH' and event.state == 1:  # Кнопка Y (может быть для стопа)
                msg.data = 'S'
        
        # Обработка оси джойстика (пример)
        elif event.ev_type == 'Absolute':
            if event.code == 'ABS_X':  # Ось X левого джойстика
                if event.state > 15000:  # Вправо
                    msg.data = 'R'
                elif event.state < -15000:  # Влево
                    msg.data = 'L'
        
        if msg.data and msg.data != self.last_command:
            self.publisher.publish(msg)
            self.last_command = msg.data
            self.node.get_logger().info(f'Command: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    controller = GamepadController()
    
    try:
        controller.process_gamepad_events()
    except KeyboardInterrupt:
        pass
    
    controller.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()