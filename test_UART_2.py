#!/usr/bin/env python3 
import rclpy
from rclpy.node  import Node
from rclpy.action  import ActionServer 
from rclpy.duration  import Duration
 
import struct 
import threading
import time 
import math
from enum import Enum 
import serial  # 使用pyserial代替smbus2 
 
from trajectory_msgs.msg  import JointTrajectory, JointTrajectoryPoint 
from control_msgs.action  import FollowJointTrajectory 
from sensor_msgs.msg  import JointState 
from geometry_msgs.msg  import Point
from blueberry_interfaces.msg  import ArmStatus 
 
# 串口通信协议定义 
class STM32Command(Enum):
    CMD_SYNC = 0xAA             # 同步命令
    CMD_SET_TRAJECTORY = 0x10   # 设置轨迹命令
    CMD_GET_STATUS = 0x20       # 获取状态命令
    CMD_EMERGENCY_STOP = 0xF0   # 紧急停止命令 
    CMD_HOME = 0x30             # 回HOME命令
 
class STM32Response(Enum):
    RESP_ACK = 0x55
    RESP_STATUS = 0x25 
    RESP_ERROR = 0xEE
 
class STM32ErrorCode(Enum):
    ERR_NONE = 0 
    ERR_INVALID_CMD = 1
    ERR_CHECKSUM = 2
    ERR_TRAJECTORY_FULL = 3
    ERR_LIMIT_EXCEEDED = 4 
    ERR_COLLISION = 5
 
class STM32CommunicationNode(Node):
    def __init__(self):
        super().__init__('stm32_communication_node')
        
        # 参数声明 
        self.declare_parameters( 
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyUSB0'),  # 串口设备路径
                ('baud_rate', 115200),           # 波特率
                ('joint_names', ['base_rotation_joint', 'joint1', 'joint2', 'joint3']),
                ('update_rate', 1.0),           # 状态更新频率(Hz)
                ('max_trajectory_points', 50),   # 最大轨迹点数 
                ('status_topic', '/arm_status'), # 状态话题
                ('max_retries', 6),              # 最大重试次数
                ('ack_timeout', 0.5),            # ACK超时时间(秒)
                ('joint_limits', [0.0, 4.188790]),  # 0° 到 240° 的弧度值 [0.0, 4.18879]
                ('test_mode', True),             # 测试模式 
                ('test_interval', 5.0),          # 测试发送间隔(秒)
                ('test_positions', [4.18879, 2.09439, 2.09439, 2.490]),  # 测试位置数据 (0-1000)
            ]
        )

        
        # 获取参数 
        self.serial_port  = self.get_parameter('serial_port').value  
        self.baud_rate  = self.get_parameter('baud_rate').value 
        self.joint_names  = self.get_parameter('joint_names').value 
        self.update_rate  = self.get_parameter('update_rate').value  
        self.max_trajectory_points  = self.get_parameter('max_trajectory_points').value 
        self.status_topic  = self.get_parameter('status_topic').value  
        self.max_retries  = self.get_parameter('max_retries').value 
        self.ack_timeout  = self.get_parameter('ack_timeout').value 
        self.joint_limits  = self.get_parameter('joint_limits').value 
        self.min_angle  = self.joint_limits[0]   # 0° 弧度值 
        self.max_angle  = self.joint_limits[1]   # 240° 弧度值
        self.angle_range  = self.max_angle  - self.min_angle 
        self.test_mode  = self.get_parameter('test_mode').value  
        self.test_interval  = self.get_parameter('test_interval').value 
        self.test_positions  = self.get_parameter('test_positions').value  
        
        # 串口对象
        self.serial_conn  = None 
        self.serial_lock  = threading.Lock()
        self.receive_buffer  = bytearray()
        
        # 当前关节状态
        self.current_joint_positions  = [0.0] * len(self.joint_names) 
        self.current_joint_velocities  = [0.0] * len(self.joint_names) 
        self.current_joint_effort  = [0.0] * len(self.joint_names) 
        
        # 系统状态 
        self.arm_connected  = False
        self.arm_ready  = False 
        self.last_status_time  = self.get_clock().now() 
        self.error_code  = 0
        self.trajectory_active  = False
        
        # 创建动作服务器
        if not self.test_mode: 
            # self.action_server  = ActionServer(
            #     self,
            #     FollowJointTrajectory,
            #     '/joint_trajectory_controller/follow_joint_trajectory',
            #     self.execute_trajectory_callback 
            # )
            self.get_logger().info(" 测试中，动作服务器暂未启用")
        else:
            self.get_logger().info(" 测试模式已启用，动作服务器未启动")
        
        # 创建关节状态发布器 
        self.joint_state_pub  = self.create_publisher( 
            JointState,
            '/joint_states',
            10 
        )
        
        # # 创建机械臂状态发布器
        self.arm_status_pub  = self.create_publisher( 
            ArmStatus,
            self.status_topic, 
            10
        )
        
        # 初始化串口连接 
        self.init_serial_connection() 

        if self.test_mode: 
            self.test_timer  = self.create_timer( 
                self.test_interval, 
                self.send_test_position,
            )
            self.get_logger().info(f" 测试定时器已启动，间隔: {self.test_interval} 秒")
        # 创建定时器用于状态更新
        
        self.update_timer  = self.create_timer( 
            1.0 / self.update_rate,  
            self.update_status,
        )
        
        # 创建测试定时器（如果启用测试模式）

        
        self.get_logger().info("STM32  Serial Communication Node initialized")
 
    def init_serial_connection(self):
        """初始化串口连接"""
        try:
            self.serial_conn  = serial.Serial(
                port=self.serial_port, 
                baudrate=self.baud_rate, 
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1  # 设置读取超时
            )
            
            if self.serial_conn.is_open: 
                self.arm_connected  = True
                self.get_logger().info(f" 串口 {self.serial_port}  连接成功，波特率: {self.baud_rate}") 
                
                # 发送同步命令
                if self.send_sync(): 
                    self.arm_ready  = True
                    self.get_logger().info("STM32  synchronized and ready")
                else:
                    self.get_logger().error("Failed  to synchronize with STM32")
            else:
                self.get_logger().error(f" 无法打开串口: {self.serial_port}") 
                
        except Exception as e:
            self.get_logger().error(f" 串口连接错误: {str(e)}")
            self.arm_connected  = False
            self.arm_ready  = False 
 
    def send_sync(self, retries=3):
        """发送同步命令"""
        for i in range(retries):
            try:
                # 发送同步命令 
                self.send_command(STM32Command.CMD_SYNC.value,  b'')
                
                # 等待ACK响应 
                response = self.read_response() 
                if response and response[0] == STM32Response.RESP_ACK.value: 
                    return True 
                
            except Exception as e:
                self.get_logger().warn(f" 同步尝试 {i+1} 失败: {str(e)}")
                time.sleep(0.1) 
        
        return False 
 
    def calculate_checksum(self, data):
        """计算校验和"""
        return sum(data) & 0xFF 
 
    def send_command(self, cmd, data):
        """通过串口发送命令到STM32"""
        if not self.serial_conn  or not self.serial_conn.is_open: 
            self.get_logger().error(" 串口未初始化或未打开!")
            return False
            
        # 构建消息: [SYNC, LENGTH, CMD, DATA..., CHECKSUM]
        length = len(data)
        message = bytearray()
        message.append(STM32Command.CMD_SYNC.value) 
        message.append(length  + 1)  # 包括CMD的长度
        message.append(cmd) 
        message.extend(data) 
        
        # 计算并添加校验和 
        checksum = self.calculate_checksum(message) 
        message.append(checksum) 
        
        try:
            with self.serial_lock: 
                self.serial_conn.write(message) 
                self.serial_conn.flush()   # 确保数据发送完成 
            return True 
        except Exception as e:
            self.get_logger().error(f" 串口写入错误: {str(e)}")
            return False
 
    def read_response(self, timeout=0.2):
        """通过串口读取STM32响应"""
        if not self.serial_conn  or not self.serial_conn.is_open: 
            return None
            
        start_time = time.time() 
        response = None
        
        # 在超时时间内尝试读取完整响应
        while (time.time()  - start_time) < timeout:
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
                self.get_logger().warn(f" 串口读取错误: {str(e)}")
                time.sleep(0.01) 
        
        return None 
 
    def parse_buffer(self):
        """从接收缓冲区解析完整消息"""
        # 查找同步头 (0xAA)
        while len(self.receive_buffer)  >= 2:
            # 找到同步头
            if self.receive_buffer[0]  != STM32Command.CMD_SYNC.value: 
                self.receive_buffer.pop(0) 
                continue 
                
            # 检查是否有足够的数据读取长度
            length = self.receive_buffer[1] 
            if len(self.receive_buffer)  < length + 2:  # +2 for SYNC and LENGTH 
                return None  # 数据不足
                
            # 提取完整消息 (包括SYNC, LENGTH和后续数据)
            full_message = self.receive_buffer[:length  + 2 + 1]  # +1 for checksum 
            
            # 验证校验和
            received_checksum = full_message[-1]
            calculated_checksum = self.calculate_checksum(full_message[:-1]) 
            
            if received_checksum == calculated_checksum:
                # 从缓冲区移除已处理的消息
                self.receive_buffer  = self.receive_buffer[length  + 3:]
                
                # 返回消息内容 (去掉SYNC和长度字节)
                return full_message[2:-1]
            else:
                self.get_logger().warn(" 校验和错误，丢弃消息")
                # 移除无效消息
                self.receive_buffer.pop(0) 
        
        return None
 
    def send_trajectory_point(self, point):
        """发送单个轨迹点到STM32，位置转换为0-1000范围"""
        # 将位置从弧度转换为0-1000范围 (0° 到 240° 对应 0-1000)
        positions = []
        for p in point.positions: 
            # 确保位置在限制范围内 
            clamped = max(self.min_angle,  min(self.max_angle,  p))
            # 转换为0-1000范围
            scaled = int(((clamped - self.min_angle)  / self.angle_range)  * 1000)
            positions.append(scaled) 
        
        # 构建数据包
        data = bytearray()
        for pos in positions:
            data.extend(struct.pack('>H',  pos))  # 大端16位无符号整数 (0-1000)

        self.get_logger().info(" 测试3")
        
        # 发送命令 
        return self.send_command_with_retry( 
            STM32Command.CMD_SET_TRAJECTORY.value, 
            data,
            expected_response=STM32Response.RESP_ACK.value  
        )
 
    def send_command_with_retry(self, cmd, data, expected_response=None, retries=None):
        """带重试机制发送命令"""
        if retries is None:
            retries = self.max_retries  
            
        # with self.serial_lock:
            for attempt in range(retries):
                try:
                    if not self.send_command(cmd, data):
                        continue
                    
                    # 如果需要响应验证
                    if expected_response is not None:
                        response = self.read_response(timeout=self.ack_timeout) 
                        if response and response[0] == expected_response:
                            return True 
                        elif response and response[0] == STM32Response.RESP_ERROR.value: 
                            if len(response) > 1:
                                error_code = response[1]
                                self.handle_error(error_code) 
                            return False 
                    else:
                        return True
                        
                except Exception as e:
                    self.get_logger().warn(f" 命令发送尝试 {attempt+1} 失败: {str(e)}")
                    time.sleep(0.05) 
                
            self.get_logger().error(f" 命令发送失败，重试 {retries} 次后仍失败")
            return False
 
    def handle_error(self, error_code):
        """处理来自STM32的错误代码"""
        self.error_code  = error_code
        try:
            error_enum = STM32ErrorCode(error_code)
            error_msg = f"STM32错误: {error_enum.name}" 
        except ValueError:
            error_msg = f"未知STM32错误代码: {error_code}"
            
        self.get_logger().error(error_msg) 
        
        # 发布状态更新 
        self.publish_arm_status() 
 
    def request_status(self):
        """请求STM32状态更新"""
        if not self.arm_ready: 
            return False
            
        try:
            # 发送命令但不等待响应
            # with self.serial_lock: 
            return self.send_command(STM32Command.CMD_GET_STATUS.value,  b'')
        except Exception as e:
            self.get_logger().error(f" 状态请求失败: {str(e)}")
            return False
 
    def parse_status_data(self, data):
        """解析状态数据，将0-1000转换回弧度值"""
        if len(data) < 1:
            return False
            
        # 状态字节: [status_flags, pos1_low, pos1_high, ...]
        status_flags = data[0]
        
        # 位置数据: 每个关节2字节 (大端无符号整数，0-1000)
        num_joints = len(self.joint_names) 
        positions = []
        
        if len(data) < 1 + num_joints * 2:
            return False
            
        for i in range(num_joints):
            idx = 1 + i * 2
            # 读取16位无符号整数
            scaled_value = struct.unpack('>H',  data[idx:idx+2])[0]
            # 转换回弧度值 (0-1000 -> 0° 到 240°)
            position_rad = self.min_angle  + (scaled_value / 1000.0) * self.angle_range 
            positions.append(position_rad) 
        
        # 更新状态
        self.current_joint_positions  = positions
        
        # 更新状态标志 
        self.trajectory_active  = bool(status_flags & 0x01)
        self.error_code  = (status_flags >> 1) & 0x0F
        
        return True
 
    def publish_joint_state(self):
        """发布关节状态"""
        msg = JointState()
        msg.header.stamp  = self.get_clock().now().to_msg() 
        msg.name  = self.joint_names  
        msg.position  = self.current_joint_positions 
        msg.velocity  = self.current_joint_velocities 
        msg.effort  = self.current_joint_effort   # 通常为0，除非有传感器
        
        self.joint_state_pub.publish(msg) 
 
    def publish_arm_status(self):
        """发布机械臂状态"""
        msg = ArmStatus()
        msg.header.stamp  = self.get_clock().now().to_msg() 
        msg.connected  = self.arm_connected  
        msg.ready  = self.arm_ready 
        msg.trajectory_active  = self.trajectory_active  
        msg.error_code  = self.error_code 
        
        self.arm_status_pub.publish(msg) 
 
    def update_status(self):
        """更新状态"""
        if not self.arm_ready: 
            # 尝试重新连接
            if not self.arm_connected: 
                self.init_serial_connection() 
            return
                
        # 定期请求状态更新 
        if self.request_status(): 
            # 读取并处理状态响应
            response = self.read_response(timeout=0.1) 
            if response:
                # 检查响应类型
                if response[0] == STM32Response.RESP_STATUS.value: 
                    # 解析状态数据 (跳过响应类型字节)
                    if self.parse_status_data(response[1:]): 
                        self.last_status_time  = self.get_clock().now() 
                elif response[0] == STM32Response.RESP_ERROR.value: 
                    self.handle_error(response[1]  if len(response) > 1 else 0)
        
        # 发布关节状态 
        self.publish_joint_state() 
        
        # 发布机械臂状态
        self.publish_arm_status() 
    
        # 处理错误状态
        if self.error_code  != STM32ErrorCode.ERR_NONE.value: 
            self.handle_error(self.error_code) 
            self.error_code  = STM32ErrorCode.ERR_NONE.value   # 重置错误码 
 
    def send_test_position(self):
        """在测试模式下发送固定的位置数据"""
        if not self.arm_ready: 
            self.get_logger().warn(" 机械臂未就绪，跳过测试位置发送")
            return
        
        # 创建轨迹点 
        point = JointTrajectoryPoint()
        point.positions  = self.test_positions   # 直接使用百分比值 (0-1000)
        point.time_from_start.sec  = 0 
        point.time_from_start.nanosec  = int(1 * 1e9)  # 1秒
        self.get_logger().info(" 测试1")
        # 发送轨迹开始标志
        if not self.send_command_with_retry( 
            STM32Command.CMD_SET_TRAJECTORY.value, 
            struct.pack('>B',  0xFF),  # 特殊标志表示轨迹开始
            expected_response=STM32Response.RESP_ACK.value  
        ):
            self.get_logger().error(" 测试轨迹启动失败")
            return 
        self.get_logger().info(" 测试2")
        # 发送轨迹点
        if not self.send_trajectory_point(point): 
            self.get_logger().error(" 测试位置发送失败")  
            return
        
        # 发送轨迹结束标志
        if not self.send_command_with_retry( 
            STM32Command.CMD_SET_TRAJECTORY.value, 
            struct.pack('>B',  0x00),  # 特殊标志表示轨迹结束 
            expected_response=STM32Response.RESP_ACK.value 
        ):
            self.get_logger().error(" 测试轨迹结束失败")
            return
        
        self.get_logger().info(f" 已发送测试位置: {self.test_positions}") 
 
    def emergency_stop(self):
        """发送紧急停止命令"""
        self.get_logger().error(" 发送紧急停止命令!!!")
        self.send_command(STM32Command.CMD_EMERGENCY_STOP.value,  b'')
        self.trajectory_active  = False 
 
    def home_arm(self):
        """发送回零命令"""
        self.get_logger().info(" 发送回零命令")
        return self.send_command_with_retry( 
            STM32Command.CMD_HOME.value, 
            b'',
            expected_response=STM32Response.RESP_ACK.value 
        )
 
    def destroy_node(self):
        """节点销毁时关闭串口连接"""
        if self.serial_conn  and self.serial_conn.is_open: 
            try:
                self.serial_conn.close() 
                self.get_logger().info(" 串口已关闭")
            except Exception as e:
                self.get_logger().error(f" 关闭串口时出错: {str(e)}")
        super().destroy_node()
 
def main(args=None):
    rclpy.init(args=args) 
    node = STM32CommunicationNode()
    
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node() 
        rclpy.shutdown() 
 
if __name__ == '__main__':
    main()