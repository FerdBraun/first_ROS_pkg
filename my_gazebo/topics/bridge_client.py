#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import socket
import struct
import zlib
import numpy as np
import msgpack
from sensor_msgs.msg import PointCloud2, PointField

class PointCloudBridgeClient(Node):
    def __init__(self):
        super().__init__('pointcloud_bridge_client')
        self.publisher_ = self.create_publisher(PointCloud2, '/remote_kinect/points', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('10.147.19.226', 12345))  # IP сервера
        self.current_header = None

    def run(self):
        while rclpy.ok():
            try:
                # Чтение длины сообщения
                raw_len = self.recv_all(4)
                if not raw_len:
                    break
                msg_len = struct.unpack('>I', raw_len)[0]

                # Получаем сериализованные данные
                data = self.recv_all(msg_len)
                deserialized = msgpack.unpackb(data, raw=False)

                # Восстанавливаем PointCloud2
                cloud_msg = self.unpack_pointcloud(deserialized)
                self.publisher_.publish(cloud_msg)

            except Exception as e:
                self.get_logger().error(f"Ошибка приёма: {e}")
                break

    def recv_all(self, n):
        data = b''
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

    def unpack_pointcloud(self, data):
        header = data.get('header', None)
        if header:
            self.current_header = header

        stamp = data['stamp']
        compressed_data = data['data']

        point_data = zlib.decompress(compressed_data)
        point_np = np.frombuffer(point_data, dtype=np.uint8)
        point_list = point_np.tolist()

        cloud_msg = PointCloud2()
        cloud_msg.header.stamp.sec = stamp['sec']
        cloud_msg.header.stamp.nanosec = stamp['nanosec']
        cloud_msg.header.frame_id = self.current_header['frame_id']
        cloud_msg.height = self.current_header['height']
        cloud_msg.width = self.current_header['width']
        cloud_msg.point_step = self.current_header['point_step']
        cloud_msg.row_step = self.current_header['row_step']
        cloud_msg.is_dense = self.current_header['is_dense']
        cloud_msg.is_bigendian = self.current_header['is_bigendian']
        cloud_msg.fields = [
            PointField(
                name=f['name'],
                offset=f['offset'],
                datatype=f['datatype'],
                count=f['count']
            ) for f in self.current_header['fields']
        ]
        cloud_msg.data = point_list
        return cloud_msg


def main(args=None):
    rclpy.init(args=args)
    client = PointCloudBridgeClient()
    client.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()