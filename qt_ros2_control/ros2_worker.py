import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time
from PySide6.QtCore import QObject, Signal, Slot

class ROS2Worker(QObject):
    # 信号定义
    image_received = Signal(object)  # 发送接收到的图像
    status_update = Signal(str)      # 发送状态更新
    error_occurred = Signal(str)     # 发送错误信息
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.executor = None
        self.thread = None
        self.running = False
        self.bridge = CvBridge()
        
    def start_ros(self):
        """启动ROS2节点"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = ROS2Node(self)
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)
            
            self.running = True
            self.thread = threading.Thread(target=self.run_ros)
            self.thread.daemon = True
            self.thread.start()
            
            self.status_update.emit("ROS2节点已启动")
            return True
        except Exception as e:
            self.error_occurred.emit(f"启动ROS2失败: {str(e)}")
            return False
    
    def run_ros(self):
        """运行ROS2执行器"""
        try:
            while self.running and rclpy.ok():
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            self.error_occurred.emit(f"ROS2执行错误: {str(e)}")
        finally:
            self.shutdown_ros()
    
    def shutdown_ros(self):
        """关闭ROS2节点"""
        self.running = False
        if self.node:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.status_update.emit("ROS2节点已关闭")
    
    def start_detection(self):
        """启动检测服务"""
        if self.node:
            self.node.call_start_service()
    
    # def stop_detection(self):
    #     """停止检测服务"""
    #     if self.node:
    #         self.node.call_stop_service()

class ROS2Node(Node):
    def __init__(self, worker):
        super().__init__('qt_control_node')
        self.worker = worker
        
        # 创建服务客户端
        self.start_client = self.create_client(Empty, 'start_detection')
        # self.stop_client = self.create_client(Empty, 'stop_detection')
        
        # 等待服务可用
        while not self.start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('start_detection服务不可用，等待中...')
        
        # while not self.stop_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('stop_detection服务不可用，等待中...')
        
        # 订阅调试图像话题
        self.image_sub = self.create_subscription(
            Image,
            'debug_image',
            self.image_callback,
            10
        )
        
        self.get_logger().info('ROS2控制节点已初始化')
    
    def image_callback(self, msg):
        """处理接收到的图像"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.worker.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 发送图像到主线程
            self.worker.image_received.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f'图像转换错误: {str(e)}')
    
    def call_start_service(self):
        """调用启动检测服务"""
        req = Empty.Request()
        future = self.start_client.call_async(req)
        future.add_done_callback(self.start_service_callback)
    
    def call_stop_service(self):
        """调用停止检测服务"""
        req = Empty.Request()
        future = self.stop_client.call_async(req)
        future.add_done_callback(self.stop_service_callback)
    
    def start_service_callback(self, future):
        """启动服务回调"""
        try:
            response = future.result()
            self.get_logger().info('检测已启动')
            self.worker.status_update.emit('检测已启动')
        except Exception as e:
            self.get_logger().error(f'启动服务调用失败: {str(e)}')
            self.worker.error_occurred.emit(f'启动服务调用失败: {str(e)}')
    
    def stop_service_callback(self, future):
        """停止服务回调"""
        try:
            response = future.result()
            self.get_logger().info('检测已停止')
            self.worker.status_update.emit('检测已停止')
        except Exception as e:
            self.get_logger().error(f'停止服务调用失败: {str(e)}')
            self.worker.error_occurred.emit(f'停止服务调用失败: {str(e)}')