#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from blueberry_interfaces.srv import PathPlan  # 替换为你的包名
import time

class PeriodicPathPlanClient(Node):
    def __init__(self):
        super().__init__('periodic_path_plan_client')
        
        # 创建服务客户端
        self.client = self.create_client(PathPlan, 'harvest_berries')
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，等待中...')
        
        # 设置定时器，每5秒发送一次请求
        self.timer = self.create_timer(5.0, self.send_request)
        self.get_logger().info('定时路径规划客户端已启动，每1秒发送一次请求...')

    def send_request(self):
        # 创建请求
        request = PathPlan.Request()
        request.start = True  # 总是发送True表示开始
        
        # 异步发送请求
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'请求成功: {response.message}')
            else:
                self.get_logger().warn(f'请求失败: {response.message}')
        except Exception as e:
            self.get_logger().error(f'服务调用失败: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    client = PeriodicPathPlanClient()
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('客户端关闭...')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()