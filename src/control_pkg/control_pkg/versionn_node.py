import rclpy
from rclpy.node  import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy 
import cv2 
import numpy as np
from cv_bridge import CvBridge 
from sensor_msgs.msg  import Image, CameraInfo 
from geometry_msgs.msg  import Point, PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener, TransformException
from blueberry_interfaces.msg  import Berry, DetectedBerries 
from blueberry_interfaces.srv  import DetectBerries 
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
                ('debug', False),  # 关闭调试模式 
                ('input_size', 640),  # 模型输入尺寸
                ('max_depth', 1.5),   # 最大有效深度(m)
                ('max_berries_per_batch', 5),  # 单次采摘最大蓝莓数 
            ]
        )
        
        # 初始化工具
        self.bridge  = CvBridge()
        self.tf_buffer  = Buffer()
        self.tf_listener  = TransformListener(self.tf_buffer,  self)
        self.color_info  = None
        self.depth_info  = None  # 深度相机内参 
        self.current_color_image  = None 
        self.current_depth_image  = None 
        self.last_detection_time  = time.time() 
        
        # 加载YOLOv8模型 
        model_path = self.get_parameter('model_path').get_parameter_value().string_value  
        self.get_logger().info(f"Loading  YOLOv8 model from: {model_path}")
        self.model  = YOLO(model_path)
        
        # 订阅话题 
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # 彩色图像订阅 
        self.color_sub  = self.create_subscription( 
            Image,
            '/camera/color/image_raw',
            self.color_image_callback, 
            qos_profile=qos 
        )
        
        # 深度图像订阅 
        self.depth_sub  = self.create_subscription( 
            Image,
            '/camera/depth/image_raw',
            self.depth_image_callback, 
            qos_profile=qos 
        )
        
        # 彩色相机内参订阅 
        self.color_info_sub  = self.create_subscription( 
            CameraInfo,
            '/camera/color/camera_info',
            self.color_info_callback, 
            10
        )
        
        # 深度相机内参订阅
        self.depth_info_sub  = self.create_subscription( 
            CameraInfo,
            '/camera/depth/camera_info',
            self.depth_info_callback, 
            10
        )
        
        # 创建检测服务
        self.detect_service  = self.create_service( 
            DetectBerries,
            'detect_berries',
            self.detect_berries_callback  
        )
        self.get_logger().info("Berry  Detection Service initialized")
 
    def color_info_callback(self, msg):
        """处理彩色相机内参信息"""
        if not self.color_info: 
            self.color_info  = msg 
            self.get_logger().info( 
                f"Color camera intrinsics: fx={msg.k[0]}, fy={msg.k[4]}, "
                f"cx={msg.k[2]}, cy={msg.k[5]}"
            )
 
    def depth_info_callback(self, msg):
        """处理深度相机内参信息"""
        if not self.depth_info: 
            self.depth_info  = msg
            self.get_logger().info( 
                f"Depth camera intrinsics: fx={msg.k[0]}, fy={msg.k[4]}, "
                f"cx={msg.k[2]}, cy={msg.k[5]}"
            )
    
    def color_image_callback(self, msg):
        """存储最新彩色图像"""
        try:
            # 转换图像格式 
            self.current_color_image  = self.bridge.imgmsg_to_cv2(msg,  desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error  processing color image: {str(e)}")
 
    def depth_image_callback(self, msg):
        """存储最新深度图像"""
        try:
            self.current_depth_image  = self.bridge.imgmsg_to_cv2(msg,  '16UC1')
        except Exception as e:
            self.get_logger().error(f"Error  processing depth image: {str(e)}")
 
    def detect_berries_callback(self, request, response):
        """服务回调函数：执行蓝莓检测并返回结果（最多5个成熟蓝莓）"""
        if not request.start:
            return 
        start_time = time.time() 
        self.get_logger().info("Received  berry detection request")
        
        # 检查是否有必要的图像数据 
        if self.current_color_image  is None:
            self.get_logger().warn("No  color image available for detection")
            return response
        if self.current_depth_image  is None:
            self.get_logger().warn("No  depth image available for detection")
            return response 
        if self.color_info  is None or self.depth_info  is None:
            self.get_logger().warn("Camera  info not available for detection")
            return response 
        
        # 使用YOLOv8进行推理
        results = self.model.predict(self.current_color_image,  imgsz=640, conf=0.5)
        
        # 解析检测结果 
        detections = []
        for result in results:
            for box in result.boxes: 
                confidence = float(box.conf) 
                if confidence < self.get_parameter('min_confidence').value: 
                    continue
                    
                # 获取边界框坐标 (xywh格式)
                x1, y1, x2, y2 = map(int, box.xyxy[0]) 
                width = x2 - x1 
                height = y2 - y1
                center_x = x1 + width // 2
                center_y = y1 + height // 2
                
                # 获取类别 (假设类别0:未成熟, 1:成熟)
                class_id = int(box.cls) 
                is_ripe = (class_id == 1)  # 只关注成熟蓝莓
                
                # 只添加成熟的蓝莓 
                if is_ripe:
                    detections.append({ 
                        'center': (center_x, center_y),
                        'bbox': [x1, y1, width, height],
                        'confidence': confidence,
                        'is_ripe': is_ripe
                    })
        
        # 处理3D位置 
        detected_berries = self.process_detections_3d(detections) 
        
        # 填充服务响应 
        response.berries.header.stamp  = self.get_clock().now().to_msg() 
        response.berries.header.frame_id  = self.get_parameter('target_frame').value  
        
        berry_id = 0
        for berry in detected_berries:
            berry_msg = Berry()
            berry_msg.id  = berry_id
            berry_id += 1 
            berry_msg.position  = berry['position']
            berry_msg.confidence  = berry['confidence']
            berry_msg.is_ripe  = berry['is_ripe']
            response.berries.berries.append(berry_msg) 
            
            # 最多返回5个成熟蓝莓
            if len(response.berries.berries)  >= self.get_parameter('max_berries_per_batch').value: 
                break
        
        response.berries.batch_size  = len(response.berries.berries) 
        
        # 记录处理时间 
        processing_time = time.time()  - start_time
        self.get_logger().info(f"Detected  {response.berries.berries}  ripe berries in {processing_time:.2f} seconds")
        
        return response
 
    def process_detections_3d(self, detections):
        """处理检测结果并计算3D位置"""
        detected_points = []
        
        # 处理每个检测到的蓝莓 
        for berry in detections:
            # 获取深度值 (中值滤波减少噪声)
            x, y = berry['center']
            depth = self.get_depth_value(self.current_depth_image,  x, y)
            
            if depth == 0 or depth > self.get_parameter('max_depth').value  * 1000:
                continue 
            
            # 计算3D位置 - 使用深度相机内参
            point_3d = self.pixel_to_3d(x,  y, depth)
            
            # 转换到目标坐标系
            point_stamped = PointStamped()
            point_stamped.header.frame_id  = self.get_parameter('camera_frame').value 
            point_stamped.header.stamp  = self.get_clock().now().to_msg() 
            point_stamped.point  = Point(x=point_3d[0], y=point_3d[1], z=point_3d[2])
            
            try:
                transform = self.tf_buffer.lookup_transform( 
                    self.get_parameter('target_frame').value, 
                    point_stamped.header.frame_id, 
                    rclpy.time.Time() 
                )
                transformed_point = do_transform_point(point_stamped, transform)
                
                # 添加到临时列表 
                detected_points.append({ 
                    'position': transformed_point.point, 
                    'confidence': berry['confidence'],
                    'is_ripe': berry['is_ripe'],
                    'image_coords': (x, y)  # 存储图像坐标用于排序 
                })
                
            except TransformException as e:
                self.get_logger().warn(f"TF  transform failed: {str(e)}")
                continue 
        
        # 按图像坐标排序（左上到右下）
        detected_points.sort(key=lambda  p: (p['image_coords'][1], p['image_coords'][0]))
        
        return detected_points
 
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
        
        return np.median(valid_depths)  if len(valid_depths) > 0 else 0
 
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