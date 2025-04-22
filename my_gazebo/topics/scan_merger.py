#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2DArray, Detection2D
from geometry_msgs.msg import PoseStamped
import torch
import numpy as np

class YOLOObstacleDetector(Node):
    def __init__(self):
        super().__init__('yolo_obstacle_detector')
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', autoshape=True).to('cpu')
        
        # Подписки
        self.sub_image = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.sub_pc = self.create_subscription(PointCloud2, '/camera/depth/points', self.pc_callback, 10)
        
        # Публикации
        self.pub_detections = self.create_publisher(Detection2DArray, '/obstacle_detections', 10)
        self.pub_obstacles = self.create_publisher(PoseStamped, '/obstacle_poses', 10)
        
        # Текущее облако точек
        self.current_pc = None

    def pc_callback(self, msg):
        self.current_pc = msg  # Сохраняем для обработки

    def image_callback(self, msg):
        if self.current_pc is None:
            return
            
        # Детекция YOLOv5
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image)
        
        detections = Detection2DArray()
        detections.header = msg.header
        
        for *xyxy, conf, cls in results.xyxy[0]:
            x_center = (xyxy[0] + xyxy[2]) / 2
            y_center = (xyxy[1] + xyxy[3]) / 2
            
            # Фильтрация по классам (0 = человек, 80 = препятствия)
            if cls == 0 or cls == 80:  
                detection = Detection2D()
                detection.bbox.center.x = float(x_center)
                detection.bbox.center.y = float(y_center)
                detection.bbox.size_x = float(xyxy[2] - xyxy[0])
                detection.bbox.size_y = float(xyxy[3] - xyxy[1])
                
                # Получаем 3D координаты из PointCloud2
                obstacle_pose = self.get_3d_pose(x_center, y_center)
                if obstacle_pose:
                    self.pub_obstacles.publish(obstacle_pose)
                
                detections.detections.append(detection)
        
        self.pub_detections.publish(detections)

    def get_3d_pose(self, x_pixel, y_pixel):
        # Конвертация PointCloud2 в numpy array (упрощенно)
        pc_data = np.frombuffer(self.current_pc.data, dtype=np.float32).reshape(-1, 4)
        
        # Берем точку по пикселям
        idx = int(y_pixel) * self.current_pc.width + int(x_pixel)
        if 0 <= idx < len(pc_data):
            x, y, z = pc_data[idx][:3]
            
            pose = PoseStamped()
            pose.header = self.current_pc.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            return pose
        return None

def main(args=None):
    rclpy.init(args=args)
    node = YOLOObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()