#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
import struct 
import threading
import time 
from enum import Enum 
import serial

# 导入Qt相关库
import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QSlider, QLabel, QPushButton, QGroupBox)
from PyQt5.QtCore import Qt, pyqtSignal

# 串口通信协议定义 
class STM32Command(Enum):
    CMD_SYNC = 0xAA             # 同步命令
    CMD_DIRECT_CONTROL = 0x10   # 直接控制命令
    CMD_EMERGENCY_STOP = 0xF0   # 紧急停止命令 
    CMD_HOME = 0x30             # 回HOME命令
 
class STM32Response(Enum):
    RESP_ACK = 0x55
    RESP_ERROR = 0xEE

# Qt界面类
class ArmControlWindow(QMainWindow):
    # 定义信号，用于在Qt线程和ROS线程之间通信
    jointPositionChanged = pyqtSignal(list)
    emergencyStopSignal = pyqtSignal()
    homeSignal = pyqtSignal()
    
    def __init__(self, joint_names):
        super().__init__()
        self.joint_names = joint_names
        self.current_positions = [500] * len(joint_names)  # 初始位置设为中间值(500)
        self.initUI()
        
    def initUI(self):
        self.setWindowTitle('机械臂实时控制')
        self.setGeometry(100, 100, 600, 400)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout()
        central_widget.setLayout(layout)
        
        # 创建关节控制组
        self.joint_sliders = []
        self.joint_labels = []
        
        for i, name in enumerate(self.joint_names):
            group = QGroupBox(f"关节 {i+1}: {name}")
            group_layout = QVBoxLayout()
            
            label = QLabel("位置: 500")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(1000)
            slider.setValue(500)
            
            # 连接信号
            slider.valueChanged.connect(lambda value, idx=i, lbl=label: 
                                       self.on_slider_changed(value, idx, lbl))
            
            group_layout.addWidget(label)
            group_layout.addWidget(slider)
            group.setLayout(group_layout)
            
            layout.addWidget(group)
            
            self.joint_sliders.append(slider)
            self.joint_labels.append(label)
        
        # 创建控制按钮组
        button_layout = QHBoxLayout()
        
        home_btn = QPushButton("回零")
        home_btn.clicked.connect(self.on_home_clicked)
        
        emergency_btn = QPushButton("紧急停止")
        emergency_btn.setStyleSheet("background-color: red; color: white;")
        emergency_btn.clicked.connect(self.on_emergency_clicked)
        
        button_layout.addWidget(home_btn)
        button_layout.addWidget(emergency_btn)
        
        layout.addLayout(button_layout)
        
        self.show()
    
    def on_slider_changed(self, value, joint_index, label):
        """滑块值改变时的回调函数"""
        label.setText(f"位置: {value}")
        self.current_positions[joint_index] = value
        # 发送所有关节的当前位置
        self.jointPositionChanged.emit(self.current_positions.copy())
    
    def on_emergency_clicked(self):
        """紧急停止按钮点击回调"""
        self.emergencyStopSignal.emit()
    
    def on_home_clicked(self):
        """回零按钮点击回调"""
        self.homeSignal.emit()
        # 重置所有滑块到中间位置
        for i, slider in enumerate(self.joint_sliders):
            slider.setValue(500)
            self.current_positions[i] = 500

# STM32通信节点
class STM32CommunicationNode(Node):
    def __init__(self, qt_app):
        super().__init__('stm32_communication_node')
        self.qt_app = qt_app
        
        # 参数声明 
        self.declare_parameters( 
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyUSB0'),
                ('baud_rate', 115200),
                ('joint_names', ['base_rotation_joint', 'joint1', 'joint2', 'joint3']),
                ('max_retries', 6),
                ('ack_timeout', 1.0),
            ]
        )

        # 获取参数 
        self.serial_port = self.get_parameter('serial_port').value  
        self.baud_rate = self.get_parameter('baud_rate').value 
        self.joint_names = self.get_parameter('joint_names').value 
        self.max_retries = self.get_parameter('max_retries').value 
        self.ack_timeout = self.get_parameter('ack_timeout').value 
        
        # 串口对象
        self.serial_conn = None 
        self.serial_lock = threading.Lock()
        self.receive_buffer = bytearray()
        
        # 系统状态 
        self.arm_connected = False
        self.arm_ready = False 
        
        # 初始化串口连接 
        self.init_serial_connection()
        
        # 连接Qt信号
        self.qt_app.window.jointPositionChanged.connect(self.send_joint_positions)
        self.qt_app.window.emergencyStopSignal.connect(self.emergency_stop)
        self.qt_app.window.homeSignal.connect(self.home_arm)
        
        self.get_logger().info("STM32 串口通信节点已初始化")
 
    def init_serial_connection(self):
        """初始化串口连接"""
        for i in range(self.max_retries):
            try:
                self.serial_conn = serial.Serial(
                    port=self.serial_port, 
                    baudrate=self.baud_rate, 
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.1
                )
                
                if self.serial_conn.is_open: 
                    self.arm_connected = True
                    self.get_logger().info(f"串口 {self.serial_port} 连接成功，波特率: {self.baud_rate}") 
                    
                    # 发送同步命令
                    if self.send_sync(): 
                        self.arm_ready = True
                        self.get_logger().info("STM32 同步成功并准备就绪")
                        return
                    else:
                        self.get_logger().error(f"同步失败 {i+1} 次")
                        continue
                else:
                    self.get_logger().error(f"无法打开串口: {self.serial_port} {i+1} 次") 
                    continue
                    
            except Exception as e:
                self.get_logger().error(f"串口连接错误: {str(e)} {i+1} 次")
                self.arm_connected = False
                self.arm_ready = False
 
    def send_sync(self, retries=3):
        """发送同步命令"""
        for i in range(retries):
            try:
                # 发送同步命令 
                self.send_command(STM32Command.CMD_SYNC.value, b'')
                
                # 等待ACK响应 
                response = self.read_response() 
                if response and response[0] == STM32Response.RESP_ACK.value: 
                    return True 
                
            except Exception as e:
                self.get_logger().warn(f"同步尝试 {i+1} 失败: {str(e)}")
                time.sleep(0.1) 
        
        return False 
 
    def calculate_checksum(self, data):
        """计算校验和"""
        return sum(data) & 0xFF 
 
    def send_command(self, cmd, data):
        """通过串口发送命令到STM32"""
        if not self.serial_conn or not self.serial_conn.is_open: 
            self.get_logger().error("串口未初始化或未打开!")
            return False
            
        # 构建消息: [SYNC, LENGTH, CMD, DATA..., CHECKSUM]
        length = len(data)
        message = bytearray()
        message.append(STM32Command.CMD_SYNC.value) 
        message.append(length + 1)  # 包括CMD的长度
        message.append(cmd) 
        message.extend(data) 
        
        # 计算并添加校验和 
        checksum = self.calculate_checksum(message) 
        message.append(checksum) 
        
        try:
            with self.serial_lock: 
                self.serial_conn.write(message) 
                self.serial_conn.flush()
            return True 
        except Exception as e:
            self.get_logger().error(f"串口写入错误: {str(e)}")
            return False

    def read_response(self, timeout=0.2):
        """通过串口读取STM32响应"""
        if not self.serial_conn or not self.serial_conn.is_open: 
            return None
            
        start_time = time.time() 
        
        # 在超时时间内尝试读取完整响应
        while (time.time() - start_time) < timeout:
            try:
                # 读取所有可用数据
                with self.serial_lock: 
                    available = self.serial_conn.in_waiting 
                    if available > 0:
                        data = self.serial_conn.read(available) 
                        self.receive_buffer.extend(data) 
                
                # 尝试从缓冲区解析完整消息 
                response = self.parse_buffer() 
                if response:
                    return response
                
                # 等待更多数据
                time.sleep(0.01) 
                
            except Exception as e:
                self.get_logger().warn(f"串口读取错误: {str(e)}")
                time.sleep(0.01) 
        
        return None 
 
    def parse_buffer(self):
        """从接收缓冲区解析完整消息"""
        # 查找同步头 (0xAA)
        while len(self.receive_buffer) >= 2:
            # 找到同步头
            if self.receive_buffer[0] != STM32Command.CMD_SYNC.value: 
                self.receive_buffer.pop(0) 
                continue 
                
            # 检查是否有足够的数据读取长度
            length = self.receive_buffer[1] 
            if len(self.receive_buffer) < length + 2:  # +2 for SYNC and LENGTH 
                return None  # 数据不足
                
            # 提取完整消息 (包括SYNC, LENGTH和后续数据)
            full_message = self.receive_buffer[:length + 2 + 1]  # +1 for checksum 
            
            # 验证校验和
            received_checksum = full_message[-1]
            calculated_checksum = self.calculate_checksum(full_message[:-1]) 
            
            if received_checksum == calculated_checksum:
                # 从缓冲区移除已处理的消息
                self.receive_buffer = self.receive_buffer[length + 3:]
                
                # 返回消息内容 (去掉SYNC和长度字节)
                return full_message[2:-1]
            else:
                self.get_logger().warn("校验和错误，丢弃消息")
                # 移除无效消息
                self.receive_buffer.pop(0) 
        
        return None

    def send_joint_positions(self, positions):
        """发送关节位置到STM32 (0-1000范围)"""
        if not self.arm_ready: 
            self.get_logger().warn("机械臂未就绪，跳过位置发送")
            return
        
        # 构建数据包
        data = bytearray()
        for pos in positions:
            # 确保位置在0-1000范围内
            clamped = max(0, min(1000, int(pos)))
            data.extend(struct.pack('>H', clamped))  # 大端16位无符号整数 (0-1000)
        
        # 发送命令 
        self.send_command(STM32Command.CMD_DIRECT_CONTROL.value, data)
 
    def emergency_stop(self):
        """发送紧急停止命令"""
        self.get_logger().error("发送紧急停止命令!!!")
        self.send_command(STM32Command.CMD_EMERGENCY_STOP.value, b'')
 
    def home_arm(self):
        """发送回零命令"""
        self.get_logger().info("发送回零命令")
        self.send_command(STM32Command.CMD_HOME.value, b'')
 
    def destroy_node(self):
        """节点销毁时关闭串口连接"""
        if self.serial_conn and self.serial_conn.is_open: 
            try:
                self.serial_conn.close() 
                self.get_logger().info("串口已关闭")
            except Exception as e:
                self.get_logger().error(f"关闭串口时出错: {str(e)}")
        super().destroy_node()

# 应用程序类，整合Qt和ROS
class ArmControlApp:
    def __init__(self, args=None):
        # 初始化ROS
        rclpy.init(args=args)
        
        # 创建Qt应用
        self.qt_app = QApplication(sys.argv)
        
        # 先创建Qt窗口
        joint_names = ['base_rotation_joint', 'joint1', 'joint2', 'joint3']
        self.window = ArmControlWindow(joint_names)
        
        # 创建ROS节点
        self.node = STM32CommunicationNode(self)
        
        # 设置退出处理
        self.qt_app.aboutToQuit.connect(self.shutdown)
        
    def run(self):
        # 启动ROS定时器处理
        timer = self.node.create_timer(0.1, self.spin_ros)
        
        # 运行Qt应用
        sys.exit(self.qt_app.exec_())
        
    def spin_ros(self):
        """处理ROS事件"""
        rclpy.spin_once(self.node, timeout_sec=0)
        
    def shutdown(self):
        """应用退出时的清理工作"""
        self.node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    app = ArmControlApp(args)
    app.run()

if __name__ == '__main__':
    main()