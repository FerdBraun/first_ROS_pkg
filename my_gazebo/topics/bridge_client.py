#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import socket
import struct
import pickle
import zlib
import numpy as np
import threading

class PointCloudBridgeClient(Node):
    def __init__(self):
        super().__init__('optimized_pointcloud_client')
        
        # ROS publisher with optimized QoS
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.publisher = self.create_publisher(
            PointCloud2,
            '/received_points',
            qos_profile
        )
        
        # Socket connection
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(('10.147.19.226', 12345))  # Server IP
        
        # State management
        self.current_header = None
        self.buffer = bytearray()
        self.lock = threading.Lock()
        
        # Start receiver thread
        self.receive_thread = threading.Thread(target=self.receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def receive_loop(self):
        while True:
            try:
                # Ensure we have length header (4 bytes)
                while len(self.buffer) < 4:
                    chunk = self.socket.recv(4096)
                    if not chunk:
                        self.get_logger().warn("Connection closed")
                        return
                    self.buffer.extend(chunk)
                
                # Extract message length
                msg_len = struct.unpack('>I', self.buffer[:4])[0]
                
                # Wait for complete message
                while len(self.buffer) < 4 + msg_len:
                    chunk = self.socket.recv(4096)
                    if not chunk:
                        self.get_logger().warn("Connection closed")
                        return
                    self.buffer.extend(chunk)
                
                # Process message
                with self.lock:
                    data = pickle.loads(self.buffer[4:4+msg_len])
                    self.process_message(data)
                
                # Remove processed message
                del self.buffer[:4+msg_len]
                
            except (ConnectionResetError, BrokenPipeError):
                self.get_logger().error("Connection lost")
                break
            except pickle.UnpicklingError as e:
                self.get_logger().error(f"Unpickling error: {e}")
                del self.buffer[:4+msg_len]
            except Exception as e:
                self.get_logger().error(f"Receive error: {e}")
                break

    def process_message(self, data):
        # Validate message structure
        if 'stamp' not in data or 'data' not in data:
            self.get_logger().error("Invalid message format")
            return
        
        # Update header if provided
        if 'header' in data:
            self.current_header = {
                'frame_id': data['header']['frame_id'],
                'height': data['header']['height'],
                'width': data['header']['width'],
                'point_step': data['header']['point_step'],
                'row_step': data['header']['row_step'],
                'is_dense': data['header']['is_dense'],
                'is_bigendian': data['header']['is_bigendian'],
                'fields': data['header']['fields']
            }
        
        if self.current_header is None:
            self.get_logger().error("No header received yet")
            return
        
        # Create PointCloud2 message
        msg = PointCloud2()
        
        # Header
        msg.header = Header()
        msg.header.frame_id = self.current_header['frame_id']
        msg.header.stamp.sec = data['stamp']['sec']
        msg.header.stamp.nanosec = data['stamp']['nanosec']
        
        # Fields
        msg.fields = [
            PointField(
                name=f['name'],
                offset=f['offset'],
                datatype=f['datatype'],
                count=f['count']
            ) for f in self.current_header['fields']
        ]
        
        # Main data
        msg.height = self.current_header['height']
        msg.width = self.current_header['width']
        msg.is_dense = self.current_header['is_dense']
        msg.is_bigendian = self.current_header['is_bigendian']
        msg.point_step = self.current_header['point_step']
        msg.row_step = self.current_header['row_step']
        
        # Decompress and set data
        try:
            msg.data = list(zlib.decompress(data['data']))
        except zlib.error as e:
            self.get_logger().error(f"Decompression error: {e}")
            return
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    client = PointCloudBridgeClient()
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()