#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class DebugImageRelay(Node):
    def __init__(self):
        super().__init__('debug_image_relay')
        
        # 订阅原始调试图像
        self.subscription = self.create_subscription(
            Image,
            'debug_image',
            self.image_callback,
            10
        )
        
        # 发布到标准话题供Qt界面使用
        self.publisher = self.create_publisher(
            Image,
            '/debug_image',
            10
        )
        
    def image_callback(self, msg):
        # 直接转发消息
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DebugImageRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()