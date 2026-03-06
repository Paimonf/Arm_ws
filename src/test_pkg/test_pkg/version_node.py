#!/usr/bin/env python3
# version_node.py
import os 
from std_srvs.srv import Empty
import logging 
logging.getLogger('NNPACK').setLevel(logging.ERROR)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy 
import cv2 
import numpy as np
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image, CameraInfo 
from geometry_msgs.msg import Point, PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener, TransformException
from blueberry_interfaces.msg import Berry, DetectedBerries, ArmStatus
from blueberry_interfaces.srv import PathPlan ,SetParams
from ultralytics import YOLO
import time 
 
class BerryDetectionNode(Node):
    def __init__(self):
        super().__init__('berry_detection_node')

        # 参数配置 
        self.declare_parameters( 
            namespace='',
            parameters=[
                ('model_path', 'best.pt'), 
                ('min_confidence', 0.65),
                ('camera_frame', 'camera_color_optical_frame'),
                ('target_frame', 'base_link'),
                ('debug', True),  # 开启调试模式以发布图像
                ('input_size', 640),  # 模型输入尺寸
                ('max_depth', 1.5),   # 最大有效深度(m)
                ('max_berries_per_batch', 5),  # 单次采摘最大蓝莓数 
                ('detection_rate', 5.0),  # 检测频率(Hz)
                ('iou_thres', 0.5)
            ]
        )
        
        # 初始化工具
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.color_info = None
        self.depth_info = None  # 深度相机内参 
        self.current_color_image = None 
        self.current_depth_image = None 
        self.results = None
        self.last_detection_time = time.time() 

        self.min_confidence=self.get_parameter('min_confidence').value
        self.iou_thres=self.get_parameter('iou_thres').value
        
        # 机械臂状态
        self.arm_status = ArmStatus()
        self.arm_status.connected = False
        self.arm_status.ready = False
        self.arm_status.trajectory_active = False
        self.arm_status.error_code = 0

        self.detection_active = False
        # 检测结果缓存
        self.detected_berries = []
        
        # 加载YOLOv8模型 
        model_path = self.get_parameter('model_path').get_parameter_value().string_value  
        self.get_logger().info(f"Loading YOLOv8 model from: {model_path}")
        self.model = YOLO(model_path)
        
        # 订阅话题 
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.berries_pub = self.create_publisher(DetectedBerries, '/detected_berries', 10)

        # 新增服务
        self.set_params_srv = self.create_service(SetParams, 'set_detection_params', self.set_params_callback)

        # 彩色图像订阅 
        self.color_sub = self.create_subscription( 
            Image,
            '/camera/color/image_raw',
            self.color_image_callback, 
            qos_profile=qos 
        )
        
        # 深度图像订阅 
        self.depth_sub = self.create_subscription( 
            Image,
            '/camera/depth/image_raw',
            self.depth_image_callback, 
            qos_profile=qos 
        )
        
        # 机械臂状态订阅
        self.arm_status_sub = self.create_subscription(
            ArmStatus,
            '/arm_status',
            self.arm_status_callback,
            10
        )
        
        # 彩色相机内参订阅 
        self.color_info_sub = self.create_subscription( 
            CameraInfo,
            '/camera/color/camera_info',
            self.color_info_callback, 
            10
        )
        
        # 深度相机内参订阅
        self.depth_info_sub = self.create_subscription( 
            CameraInfo,
            '/camera/depth/camera_info',
            self.depth_info_callback, 
            10
        )

        self.start_detection_service = self.create_service(
            Empty, 
            'start_detection', 
            self.start_detection_callback
        )
        
        self.stop_detection_service = self.create_service(
            Empty, 
            'stop_detection', 
            self.stop_detection_callback
        )        

        # 创建路径规划服务客户端
        self.path_plan_client = self.create_client(PathPlan, 'path_plan')
        
        # 创建发布者 - 发布带检测框的图像
        self.debug_image_pub = self.create_publisher(Image, 'debug_image', 10)
        
        # 创建定时器用于实时检测
        detection_rate = self.get_parameter('detection_rate').value
        self.detection_timer = self.create_timer(1.0 / detection_rate, self.real_time_detection_callback)
        
        self.get_logger().info("Berry Detection Node initialized with real-time detection")
 
    def start_detection_callback(self, request, response):
        """处理启动检测服务请求"""
        self.detection_active = True
        self.get_logger().info("检测已启动")
        return response
    
    def stop_detection_callback(self, request, response):
        """处理启动检测服务请求"""
        self.detection_active = False
        self.get_logger().info("检测已关闭")
        return response
        
    def arm_status_callback(self, msg):
        """处理机械臂状态更新"""
        self.arm_status = msg
        
        # 如果机械臂就绪且空闲，并且有检测到的蓝莓，则调用路径规划服务
        if (self.arm_status.ready and 
            not self.arm_status.trajectory_active and 
            self.arm_status.error_code == 0 and
            self.detected_berries and self.detection_active):
            self.get_logger().info("Detected berries ready for path planning")  
            self.call_path_plan_service()
 
    def set_params_callback(self, request, response):
            try:
                self.set_parameters([
                    rclpy.parameter.Parameter('min_confidence', rclpy.Parameter.Type.DOUBLE, request.confidence),
                    rclpy.parameter.Parameter('iou_thres', rclpy.Parameter.Type.DOUBLE, request.iou)
                ])
                self.min_confidence=self.get_parameter("min_confidence").value
                self.iou_thres=self.get_parameter("iou_thres").value
                response.success = True
                response.message = "Parameters updated"
                self.get_logger().info(f"设置置信度：{self.min_confidence}")
                self.get_logger().info(f"设置交并比阈值：{self.iou_thres}")
            except Exception as e:
                response.success = False
                response.message = str(e)
            return response

    def color_info_callback(self, msg):
        """处理彩色相机内参信息"""
        if not self.color_info: 
            self.color_info = msg 
            self.get_logger().info( 
                f"Color camera intrinsics: fx={msg.k[0]}, fy={msg.k[4]}, "
                f"cx={msg.k[2]}, cy={msg.k[5]}"
            )
 
    def depth_info_callback(self, msg):
        """处理深度相机内参信息"""
        if not self.depth_info: 
            self.depth_info = msg
            self.get_logger().info( 
                f"Depth camera intrinsics: fx={msg.k[0]}, fy={msg.k[4]}, "
                f"cx={msg.k[2]}, cy={msg.k[5]}"
            )
    
    def color_image_callback(self, msg):
        """存储最新彩色图像"""
        try:
            # 转换图像格式 
            self.current_color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error processing color image: {str(e)}")
 
    def depth_image_callback(self, msg):
        """存储最新深度图像"""
        try:
            self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")
    
    def real_time_detection_callback(self):
        """实时检测回调函数"""
        # 检查是否有必要的图像数据 
        if self.current_color_image is None:
            self.get_logger().warn("No color image available for detection")
            return
        if self.current_depth_image is None:
            self.get_logger().warn("No depth image available for detection")
            return 
        if self.color_info is None or self.depth_info is None:
            self.get_logger().warn("Camera info not available for detection")
            return 
        
        # 使用YOLOv8进行推理
        self.results = self.model.predict(self.current_color_image, imgsz=640, conf=0.5)
        
        # 解析检测结果 
        detections = []
        for result in self.results:
            for box in result.boxes: 
                confidence = float(box.conf) 
                if confidence < self.min_confidence: 
                    continue
                    
                # 获取边界框坐标 (xywh格式)
                x1, y1, x2, y2 = map(int, box.xyxy[0]) 
                width = x2 - x1 
                height = y2 - y1
                center_x = x1 + width // 2
                center_y = y1 + height // 2
                
                # 获取类别 (假设类别0:未成熟, 1:成熟)
                class_id = int(box.cls) 
                # self.get_logger().info(f"Detected berry: {class_id}, confidence: {confidence}")
                is_ripe = (class_id == 0)  # 只关注成熟蓝莓
                
                # 只添加成熟的蓝莓 
                if is_ripe:
                    detections.append({ 
                        'center': (center_x, center_y),
                        'bbox': [x1, y1, width, height],
                        'confidence': confidence,
                        'is_ripe': is_ripe
                    })
        
        # 处理3D位置 
        self.detected_berries = self.process_detections_3d(detections) 
        
        
        # 发布带检测框的图像
        if self.get_parameter('debug').value:
             
            try:
                debug_image = self.results[0].plot()
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
                debug_msg.header.stamp = self.get_clock().now().to_msg()
                self.debug_image_pub.publish(debug_msg)
                self.get_logger().info("蓝莓识别图像已发布")
                # 发布检测结果
                berries_msg = DetectedBerries()
                berries_msg.header.stamp = self.get_clock().now().to_msg()
                berries_msg.header.frame_id = self.get_parameter('target_frame').value
                for berry in self.detected_berries:
                    b = Berry()
                    b.id = len(berries_msg.berries)
                    b.position = berry['position']
                    b.confidence = berry['confidence']
                    b.is_ripe = berry['is_ripe']
                    b.size = berry['size']
                    berries_msg.berries.append(b)
                berries_msg.batch_size = len(berries_msg.berries)
                self.berries_pub.publish(berries_msg)
                self.get_logger().info("蓝莓识别信息已发布")
            except Exception as e:
                self.get_logger().error(f"Error publishing debug image: {str(e)}")

    def call_path_plan_service(self):
        """调用路径规划服务"""
        if not self.path_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Path plan service not available")
            return
        
        if not self.detected_berries:
            return

        # 发布检测结果
        berrys = self.detected_berries
        
        # 创建请求
        request = PathPlan.Request()
        request.berries.header.stamp = self.get_clock().now().to_msg()
        request.berries.header.frame_id = self.get_parameter('target_frame').value

        berry_id = 0
        for berry in berrys:
            berry_msg = Berry()
            berry_msg.id  = berry_id
            berry_id += 1 
            berry_msg.position  = berry['position']
            berry_msg.confidence  = berry['confidence']
            berry_msg.is_ripe  = berry['is_ripe']
            berry_msg.size = berry['size']
            request.berries.berries.append(berry_msg)
            if len(request.berries.berries) >= self.get_parameter('max_berries_per_batch').value: 
                break

        request.berries.batch_size = len(request.berries.berries)
        self.get_logger().info(f"Sending {len(request.berries.berries)} berries to path plan service")
            
        # 异步调用服务
        future = self.path_plan_client.call_async(request)
        self.get_logger().info("Waiting for path plan service response")
        future.add_done_callback(self.path_plan_callback)
        
        self.get_logger().info("Called path plan service")
 
    def path_plan_callback(self, future):
        """处理路径规划服务响应"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Path planning successful")
                # 移除已处理的蓝莓
                if self.detected_berries:
                    self.detected_berries.pop(0)
            else:
                self.get_logger().warn(f"Path planning failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
 
    def process_detections_3d(self, detections):
        """处理检测结果并计算3D位置及尺寸"""
        detected_points = []
        
        for berry in detections:
            x, y = berry['center']
            depth = self.get_depth_value(self.current_depth_image, x, y)
            if depth == 0 or depth > self.get_parameter('max_depth').value * 1000:
                continue
            
            # 计算3D位置
            point_3d = self.pixel_to_3d(x, y, depth)
            
            # 估算蓝莓实际尺寸（直径）
            bbox_w = berry['bbox'][2]   # 边界框宽度（像素）
            bbox_h = berry['bbox'][3]   # 边界框高度（像素）
            size = self.pixel_to_size(bbox_w, bbox_h, depth)
            self.get_logger().info(f"蓝莓大小:{size}")
            
            # 转换到目标坐标系（位置转换）
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.get_parameter('camera_frame').value
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point = Point(x=point_3d[0], y=point_3d[1], z=point_3d[2])
            
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.get_parameter('target_frame').value,
                    point_stamped.header.frame_id,
                    rclpy.time.Time()
                )
                transformed_point = do_transform_point(point_stamped, transform)
                
                detected_points.append({
                    'position': transformed_point.point,
                    'confidence': berry['confidence'],
                    'is_ripe': berry['is_ripe'],
                    'size': size,                     # 新增尺寸字段
                    'image_coords': (x, y)
                })
            except TransformException as e:
                self.get_logger().warn(f"TF transform failed: {str(e)}")
                continue
        
        detected_points.sort(key=lambda p: (p['image_coords'][1], p['image_coords'][0]))
        return detected_points
 
    
    def pixel_to_size(self, pixel_width, pixel_height, depth_mm):
        """
        将像素尺寸转换为实际物理尺寸（直径）
        :param pixel_width: 边界框宽度（像素）
        :param pixel_height: 边界框高度（像素）
        :param depth_mm: 深度值（毫米）
        :return: 实际直径（米）
        """
        if not self.depth_info:
            return 0.0
        
        fx = self.depth_info.k[0]  # 水平方向焦距（像素）
        fy = self.depth_info.k[4]  # 垂直方向焦距（像素）
        
        depth_m = depth_mm / 1000.0
        
        # 分别计算宽度和高度对应的实际尺寸
        real_width = (pixel_width * depth_m) / fx
        real_height = (pixel_height * depth_m) / fy
        
        # 取平均值作为直径（假设蓝莓近似圆形）
        diameter = (real_width + real_height) / 2.0
        return (diameter*1000)

    def get_depth_value(self, depth_image, x, y, kernel_size=5):
        """获取深度值 (使用中值滤波减少噪声)"""
        h, w = depth_image.shape 
        half_k = kernel_size // 2
        
        # 确保不越界
        x_min = max(0, x - half_k)
        x_max = min(w, x + half_k + 1)
        y_min = max(0, y - half_k)
        y_max = min(h, y + half_k + 1)
        
        # 提取ROI并计算中值
        roi = depth_image[y_min:y_max, x_min:x_max]
        valid_depths = roi[roi > 0]
        
        return np.median(valid_depths) if len(valid_depths) > 0 else 0
 
    def pixel_to_3d(self, u, v, depth):
        """将像素坐标转换为3D点 (相机坐标系) - 使用深度相机内参"""
        if not self.depth_info: 
            return (0, 0, 0)
            
        # 使用深度相机内参 
        fx = self.depth_info.k[0] 
        fy = self.depth_info.k[4] 
        cx = self.depth_info.k[2] 
        cy = self.depth_info.k[5] 
        
        # 修正的坐标转换公式 
        z = depth / 1000.0  # 转换为米 
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy 
        
        # 返回修正后的坐标 (x, y, z)
        return (x, y, z)
 
def main(args=None):
    rclpy.init(args=args) 
    node = BerryDetectionNode()
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 
 
if __name__ == '__main__':
    main()