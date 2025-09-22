#!/usr/bin/env python3
import sys
import os
import threading
import time
from datetime import datetime

# 设置环境变量以避免与ROS2冲突
os.environ['QT_API'] = 'pyqt5'

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from blueberry_interfaces.msg import ArmStatus, DetectedBerries
from blueberry_interfaces.srv import PathPlan

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QTextEdit, QGroupBox, QGridLayout, 
                             QTabWidget, QSpinBox, QDoubleSpinBox, QComboBox, QCheckBox)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap, QFont

# ROS2节点运行在单独线程中
class ROS2NodeThread(QObject, Node):
    update_signal = pyqtSignal(str, object)  # 信号用于更新UI
    
    def __init__(self):
        QObject.__init__(self)
        Node.__init__(self, 'qt_interface_node')
        
        # 初始化CV桥接
        self.bridge = CvBridge()
        
        # 订阅者
        self.image_sub = self.create_subscription(
            Image, '/debug_image', self.image_callback, 10
        )
        self.arm_status_sub = self.create_subscription(
            ArmStatus, '/arm_status', self.arm_status_callback, 10
        )
        self.berries_sub = self.create_subscription(
            DetectedBerries, '/detected_berries', self.berries_callback, 10
        )
        
        # 服务客户端
        self.path_plan_client = self.create_client(PathPlan, 'path_plan')
        
        self.current_image = None
        self.arm_status = None
        self.detected_berries = None
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
            self.update_signal.emit('image', cv_image)
        except Exception as e:
            self.get_logger().error(f"图像转换错误: {str(e)}")
    
    def arm_status_callback(self, msg):
        self.arm_status = msg
        self.update_signal.emit('arm_status', msg)
    
    def berries_callback(self, msg):
        self.detected_berries = msg
        self.update_signal.emit('berries', msg)
    
    def call_path_plan_service(self, berries):
        if not self.path_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("路径规划服务不可用")
            return False
            
        request = PathPlan.Request()
        request.berries = berries
        
        future = self.path_plan_client.call_async(request)
        # 可以添加回调处理结果
        return True

# 主窗口类
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("机械臂蓝莓采摘控制系统")
        self.setGeometry(100, 100, 1200, 800)
        
        # 初始化ROS2
        self.ros_thread = None
        self.ros_node = None
        self.executor = None
        
        self.init_ui()
        self.init_ros()
        
    def init_ui(self):
        # 中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout(central_widget)
        
        # 左侧控制面板
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        control_layout.setAlignment(Qt.AlignTop)
        control_panel.setMaximumWidth(300)
        
        # 系统控制组
        system_group = QGroupBox("系统控制")
        system_layout = QVBoxLayout(system_group)
        
        self.start_btn = QPushButton("启动系统")
        self.start_btn.clicked.connect(self.start_system)
        system_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("停止系统")
        self.stop_btn.clicked.connect(self.stop_system)
        self.stop_btn.setEnabled(False)
        system_layout.addWidget(self.stop_btn)
        
        self.emergency_btn = QPushButton("紧急停止")
        self.emergency_btn.clicked.connect(self.emergency_stop)
        self.emergency_btn.setStyleSheet("background-color: red; color: white;")
        system_layout.addWidget(self.emergency_btn)
        
        control_layout.addWidget(system_group)
        
        # 参数设置组
        params_group = QGroupBox("参数设置")
        params_layout = QGridLayout(params_group)
        
        params_layout.addWidget(QLabel("最小置信度:"), 0, 0)
        self.confidence_spin = QDoubleSpinBox()
        self.confidence_spin.setRange(0.1, 1.0)
        self.confidence_spin.setValue(0.65)
        self.confidence_spin.setSingleStep(0.05)
        params_layout.addWidget(self.confidence_spin, 0, 1)
        
        params_layout.addWidget(QLabel("最大采摘数量:"), 1, 0)
        self.max_berries_spin = QSpinBox()
        self.max_berries_spin.setRange(1, 10)
        self.max_berries_spin.setValue(5)
        params_layout.addWidget(self.max_berries_spin, 1, 1)
        
        params_layout.addWidget(QLabel("检测频率:"), 2, 0)
        self.detection_rate_spin = QDoubleSpinBox()
        self.detection_rate_spin.setRange(0.1, 10.0)
        self.detection_rate_spin.setValue(5.0)
        self.detection_rate_spin.setSingleStep(0.5)
        params_layout.addWidget(self.detection_rate_spin, 2, 1)
        
        control_layout.addWidget(params_group)
        
        # 机械臂控制组
        arm_control_group = QGroupBox("机械臂控制")
        arm_control_layout = QVBoxLayout(arm_control_group)
        
        self.home_btn = QPushButton("回零位置")
        self.home_btn.clicked.connect(self.home_arm)
        arm_control_layout.addWidget(self.home_btn)
        
        self.manual_control_btn = QPushButton("手动控制")
        self.manual_control_btn.clicked.connect(self.toggle_manual_control)
        arm_control_layout.addWidget(self.manual_control_btn)
        
        control_layout.addWidget(arm_control_group)
        
        # 状态显示组
        status_group = QGroupBox("系统状态")
        status_layout = QVBoxLayout(status_group)
        
        self.connection_status = QLabel("未连接")
        self.connection_status.setStyleSheet("color: red; font-weight: bold;")
        status_layout.addWidget(self.connection_status)
        
        self.arm_status_label = QLabel("机械臂状态: 未知")
        status_layout.addWidget(self.arm_status_label)
        
        self.berry_count_label = QLabel("检测到蓝莓: 0")
        status_layout.addWidget(self.berry_count_label)
        
        control_layout.addWidget(status_group)
        
        # 右侧显示区域
        display_area = QTabWidget()
        
        # 图像显示标签
        image_tab = QWidget()
        image_layout = QVBoxLayout(image_tab)
        
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setText("等待图像数据...")
        image_layout.addWidget(self.image_label)
        
        display_area.addTab(image_tab, "图像显示")
        
        # 日志显示标签
        log_tab = QWidget()
        log_layout = QVBoxLayout(log_tab)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        display_area.addTab(log_tab, "系统日志")
        
        # 添加到主布局
        main_layout.addWidget(control_panel)
        main_layout.addWidget(display_area, 1)
        
        # 状态栏
        self.statusBar().showMessage("就绪")
        
        # 定时器用于更新UI
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(100)  # 10Hz更新频率
        
    def init_ros(self):
        # 在单独线程中初始化ROS2
        self.ros_thread = threading.Thread(target=self.ros_thread_func)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
    def ros_thread_func(self):
        rclpy.init()
        self.ros_node = ROS2NodeThread()
        self.ros_node.update_signal.connect(self.handle_ros_update)
        
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.ros_node)
        
        try:
            self.executor.spin()
        finally:
            self.executor.shutdown()
            self.ros_node.destroy_node()
            rclpy.shutdown()
            
    def handle_ros_update(self, msg_type, data):
        if msg_type == 'image':
            self.current_image = data
        elif msg_type == 'arm_status':
            self.current_arm_status = data
        elif msg_type == 'berries':
            self.current_berries = data
            
    def update_ui(self):
        # 更新图像显示
        if hasattr(self, 'current_image') and self.current_image is not None:
            height, width, channel = self.current_image.shape
            bytes_per_line = 3 * width
            q_img = QImage(self.current_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(q_img))
            
        # 更新状态显示
        if hasattr(self, 'current_arm_status') and self.current_arm_status is not None:
            status = self.current_arm_status
            if status.connected:
                self.connection_status.setText("已连接")
                self.connection_status.setStyleSheet("color: green; font-weight: bold;")
            else:
                self.connection_status.setText("未连接")
                self.connection_status.setStyleSheet("color: red; font-weight: bold;")
                
            status_text = f"机械臂状态: {'就绪' if status.ready else '忙碌'}"
            if status.trajectory_active:
                status_text += " (运动中)"
            self.arm_status_label.setText(status_text)
            
        # 更新蓝莓计数
        if hasattr(self, 'current_berries') and self.current_berries is not None:
            count = len(self.current_berries.berries)
            self.berry_count_label.setText(f"检测到蓝莓: {count}")
            
    def start_system(self):
        self.log("启动系统中...")
        # 这里应该启动ROS2 launch文件
        # 可以使用subprocess调用ros2 launch
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.statusBar().showMessage("系统运行中")
        
    def stop_system(self):
        self.log("停止系统中...")
        # 停止ROS2节点
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.statusBar().showMessage("系统已停止")
        
    def emergency_stop(self):
        self.log("紧急停止!")
        # 发送紧急停止命令
        if self.ros_node:
            # 这里应该调用紧急停止服务或发布话题
            pass
            
    def home_arm(self):
        self.log("机械臂回零...")
        # 发送回零命令
        
    def toggle_manual_control(self):
        if self.manual_control_btn.text() == "手动控制":
            self.manual_control_btn.setText("退出手动")
            self.log("进入手动控制模式")
        else:
            self.manual_control_btn.setText("手动控制")
            self.log("退出手动控制模式")
            
    def log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        # 保持日志在最新位置
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
        
    def closeEvent(self, event):
        # 清理资源
        if self.executor:
            self.executor.shutdown()
        if self.ros_node:
            self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()