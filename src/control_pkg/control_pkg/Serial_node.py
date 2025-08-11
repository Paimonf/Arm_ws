#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration

import struct
import threading
import time
import math
from enum import Enum
from smbus2 import SMBus, i2c_msg

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from blueberry_interfaces.msg import ArmStatus

# I2C通信协议定义
class STM32Command(Enum):
    CMD_SYNC = 0xAA
    CMD_SET_TRAJECTORY = 0x10
    CMD_GET_STATUS = 0x20
    CMD_EMERGENCY_STOP = 0xF0
    CMD_HOME = 0x30

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
                ('i2c_bus', 1),                  # I2C总线号
                ('i2c_address', 0x50),           # STM32 I2C地址
                ('joint_names', ['base_rotation_joint', 'joint1', 'joint2', 'joint3']),
                ('update_rate', 20.0),           # 状态更新频率(Hz)
                ('max_trajectory_points', 50),   # 最大轨迹点数
                ('status_topic', '/arm_status'), # 状态话题
                ('max_retries', 3),              # 最大重试次数
                ('ack_timeout', 0.2),            # ACK超时时间(秒)
                ('position_scale', 1000.0),      # 位置值缩放因子(弧度->毫弧度)
                ('velocity_scale', 1000.0),      # 速度值缩放因子(弧度/秒->毫弧度/秒)
                ('max_i2c_payload', 32),         # I2C最大有效载荷
            ]
        )
        
        # 获取参数
        self.i2c_bus_num = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.joint_names = self.get_parameter('joint_names').value
        self.update_rate = self.get_parameter('update_rate').value
        self.max_trajectory_points = self.get_parameter('max_trajectory_points').value
        self.status_topic = self.get_parameter('status_topic').value
        self.max_retries = self.get_parameter('max_retries').value
        self.ack_timeout = self.get_parameter('ack_timeout').value
        self.position_scale = self.get_parameter('position_scale').value
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.max_i2c_payload = self.get_parameter('max_i2c_payload').value
        
        # I2C对象
        self.i2c_bus = None
        self.i2c_lock = threading.Lock()
        
        # 当前关节状态
        self.current_joint_positions = [0.0] * len(self.joint_names)
        self.current_joint_velocities = [0.0] * len(self.joint_names)
        self.current_joint_effort = [0.0] * len(self.joint_names)
        
        # 系统状态
        self.arm_connected = False
        self.arm_ready = False
        self.last_status_time = self.get_clock().now()
        self.error_code = 0
        self.trajectory_active = False
        
        # 创建动作服务器
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.execute_trajectory_callback
        )
        
        # 创建关节状态发布器
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # 创建机械臂状态发布器
        self.arm_status_pub = self.create_publisher(
            ArmStatus,
            self.status_topic,
            10
        )
        
        # 初始化I2C连接
        self.init_i2c_connection()
        
        # 创建定时器用于状态更新
        self.update_timer = self.create_timer(
            1.0 / self.update_rate, 
            self.update_status
        )
        
        self.get_logger().info("STM32 I2C Communication Node initialized")

    def init_i2c_connection(self):
        """初始化I2C连接"""
        try:
            with self.i2c_lock:
                self.i2c_bus = SMBus(self.i2c_bus_num)
                self.arm_connected = True
                self.get_logger().info(f"Connected to I2C bus {self.i2c_bus_num}")
                
                # 发送同步命令
                if self.send_sync():
                    self.arm_ready = True
                    self.get_logger().info("STM32 synchronized and ready")
                else:
                    self.get_logger().error("Failed to synchronize with STM32")
                
        except Exception as e:
            self.get_logger().error(f"I2C connection error: {str(e)}")
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
                self.get_logger().warn(f"Sync attempt {i+1} failed: {str(e)}")
                time.sleep(0.1)
        
        return False

    def calculate_checksum(self, data):
        """计算校验和"""
        return sum(data) & 0xFF

    def send_command(self, cmd, data):
        """通过I2C发送命令到STM32"""
        if not self.i2c_bus:
            self.get_logger().error("I2C bus not initialized")
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
        
        # 检查长度是否超过I2C限制
        if len(message) > self.max_i2c_payload:
            self.get_logger().error(f"Message too long for I2C: {len(message)} > {self.max_i2c_payload}")
            return False
        
        try:
            # 使用i2c_msg进行高效写入
            write_msg = i2c_msg.write(self.i2c_address, message)
            with self.i2c_lock:
                self.i2c_bus.i2c_rdwr(write_msg)
            return True
        except Exception as e:
            self.get_logger().error(f"I2C write error: {str(e)}")
            return False

    def read_response(self, timeout=0.2):
        """通过I2C读取STM32响应"""
        if not self.i2c_bus:
            return None
            
        start_time = time.time()
        
        # 首先尝试读取1字节以检查是否有数据
        while (time.time() - start_time) < timeout:
            try:
                # 尝试读取1字节
                read_msg = i2c_msg.read(self.i2c_address, 1)
                with self.i2c_lock:
                    self.i2c_bus.i2c_rdwr(read_msg)
                
                if len(list(read_msg)) > 0:
                    # 如果收到第一个字节，继续读取完整消息
                    first_byte = list(read_msg)[0]
                    
                    # 检查同步字节
                    if first_byte != STM32Command.CMD_SYNC.value:
                        # 如果不是同步字节，继续等待
                        time.sleep(0.001)
                        continue
                    
                    # 读取长度字节
                    len_msg = i2c_msg.read(self.i2c_address, 1)
                    with self.i2c_lock:
                        self.i2c_bus.i2c_rdwr(len_msg)
                    
                    if len(list(len_msg)) > 0:
                        length = list(len_msg)[0]
                        
                        # 读取剩余消息 (长度包括校验和)
                        data_msg = i2c_msg.read(self.i2c_address, length + 1)
                        with self.i2c_lock:
                            self.i2c_bus.i2c_rdwr(data_msg)
                        
                        data_bytes = list(data_msg)
                        if len(data_bytes) == length + 1:
                            # 构建完整消息: SYNC + LENGTH + DATA + CHECKSUM
                            full_message = bytearray([first_byte, length]) + bytearray(data_bytes)
                            
                            # 验证校验和
                            received_checksum = full_message[-1]
                            calculated_checksum = self.calculate_checksum(full_message[:-1])
                            
                            if received_checksum == calculated_checksum:
                                # 返回消息内容 (去掉SYNC和长度字节)
                                return full_message[2:-1]
                            else:
                                self.get_logger().warn("Checksum error in response")
                                return None
            except Exception as e:
                self.get_logger().warn(f"I2C read error: {str(e)}")
                time.sleep(0.005)
        
        return None

    def send_trajectory_point(self, point):
        """发送单个轨迹点到STM32"""
        # 转换位置和速度值 (弧度 -> 毫弧度)
        positions = [int(p * self.position_scale) for p in point.positions]
        velocities = [int(v * self.velocity_scale) for v in point.velocities] if point.velocities else [0] * len(positions)
        
        # 获取时间 (秒 -> 毫秒)
        time_ms = int((point.time_from_start.sec * 1000) + 
                      (point.time_from_start.nanosec / 1e6))
        
        # 构建数据包
        data = bytearray()
        for pos in positions:
            data.extend(struct.pack('>h', pos))  # 大端16位有符号整数
        for vel in velocities:
            data.extend(struct.pack('>h', vel))  # 大端16位有符号整数
        data.extend(struct.pack('>I', time_ms))  # 大端32位无符号整数
        
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
                self.get_logger().warn(f"Command send attempt {attempt+1} failed: {str(e)}")
                time.sleep(0.05)
                
        self.get_logger().error(f"Failed to send command after {retries} attempts")
        return False

    def handle_error(self, error_code):
        """处理来自STM32的错误代码"""
        self.error_code = error_code
        try:
            error_enum = STM32ErrorCode(error_code)
            error_msg = f"STM32 error: {error_enum.name}"
        except ValueError:
            error_msg = f"STM32 unknown error code: {error_code}"
            
        self.get_logger().error(error_msg)
        
        # 发布状态更新
        self.publish_arm_status()

    def execute_trajectory_callback(self, goal_handle):
        """执行轨迹回调函数"""
        if not self.arm_ready:
            goal_handle.abort()
            self.get_logger().error("Cannot execute trajectory - arm not ready")
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Arm not ready"
            return result
        
        trajectory = goal_handle.request.trajectory
        
        # 验证关节名称
        if trajectory.joint_names != self.joint_names:
            goal_handle.abort()
            self.get_logger().error("Joint names in trajectory do not match expected")
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = "Joint names mismatch"
            return result
        
        # 检查轨迹点数
        if len(trajectory.points) == 0:
            goal_handle.abort()
            self.get_logger().error("Empty trajectory received")
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Empty trajectory"
            return result
        
        # 检查最大点数
        if len(trajectory.points) > self.max_trajectory_points:
            goal_handle.abort()
            self.get_logger().error(f"Trajectory has too many points ({len(trajectory.points)} > {self.max_trajectory_points})")
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Too many trajectory points"
            return result
        
        self.get_logger().info(f"Received trajectory with {len(trajectory.points)} points")
        
        # 发送轨迹开始标志
        if not self.send_command_with_retry(
            STM32Command.CMD_SET_TRAJECTORY.value,
            struct.pack('>B', 0xFF),  # 特殊标志表示轨迹开始
            expected_response=STM32Response.RESP_ACK.value
        ):
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.FAILED
            result.error_string = "Failed to start trajectory"
            return result
        
        # 发送轨迹点
        for i, point in enumerate(trajectory.points):
            if not self.send_trajectory_point(point):
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.FAILED
                result.error_string = f"Failed to send trajectory point {i+1}"
                return result
            
            # 发布反馈
            feedback = FollowJointTrajectory.Feedback()
            feedback.actual = point
            feedback.desired = point
            goal_handle.publish_feedback(feedback)
        
        # 发送轨迹结束标志
        if not self.send_command_with_retry(
            STM32Command.CMD_SET_TRAJECTORY.value,
            struct.pack('>B', 0x00),  # 特殊标志表示轨迹结束
            expected_response=STM32Response.RESP_ACK.value
        ):
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.FAILED
            result.error_string = "Failed to end trajectory"
            return result
        
        # 标记轨迹激活
        self.trajectory_active = True
        
        # 等待轨迹完成
        start_time = self.get_clock().now()
        last_point_time = trajectory.points[-1].time_from_start
        timeout_duration = Duration(seconds=last_point_time.sec + last_point_time.nanosec / 1e9 + 2.0)  # 加2秒容差
        
        while self.trajectory_active and rclpy.ok():
            if self.get_clock().now() - start_time > timeout_duration:
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = "Trajectory execution timed out"
                return result
            
            time.sleep(0.1)
        
        # 轨迹成功完成
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def request_status(self):
        """请求STM32状态更新"""
        if not self.arm_ready:
            return False
            
        try:
            # 发送命令但不等待响应
            with self.i2c_lock:
                return self.send_command(STM32Command.CMD_GET_STATUS.value, b'')
        except Exception as e:
            self.get_logger().error(f"Status request failed: {str(e)}")
            return False

    def parse_status_data(self, data):
        """解析状态数据"""
        if len(data) < 1:
            return False
            
        # 状态字节: [status_flags, pos1_low, pos1_high, ...]
        status_flags = data[0]
        
        # 位置数据: 每个关节2字节 (大端有符号整数)
        num_joints = len(self.joint_names)
        positions = []
        
        if len(data) < 1 + num_joints * 2:
            return False
            
        for i in range(num_joints):
            idx = 1 + i * 2
            pos = struct.unpack('>h', data[idx:idx+2])[0]
            positions.append(pos / self.position_scale)  # 毫弧度 -> 弧度
        
        # 速度数据 (可选)
        velocities = []
        if len(data) >= 1 + num_joints * 4:
            for i in range(num_joints):
                idx = 1 + num_joints * 2 + i * 2
                vel = struct.unpack('>h', data[idx:idx+2])[0]
                velocities.append(vel / self.velocity_scale)  # 毫弧度/秒 -> 弧度/秒
        
        # 更新状态
        self.current_joint_positions = positions
        if velocities:
            self.current_joint_velocities = velocities
        
        # 更新状态标志
        self.trajectory_active = bool(status_flags & 0x01)
        self.error_code = (status_flags >> 1) & 0x0F
        
        return True

    def publish_joint_state(self):
        """发布关节状态"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_joint_positions
        msg.velocity = self.current_joint_velocities
        msg.effort = self.current_joint_effort  # 通常为0，除非有传感器
        
        self.joint_state_pub.publish(msg)

    def publish_arm_status(self):
        """发布机械臂状态"""
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = self.arm_connected
        msg.ready = self.arm_ready
        msg.trajectory_active = self.trajectory_active
        msg.error_code = self.error_code
        
        self.arm_status_pub.publish(msg)

    def update_status(self):
        """更新状态"""
        if not self.arm_ready:
            # 尝试重新连接
            if not self.arm_connected:
                self.init_i2c_connection()
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
                        self.last_status_time = self.get_clock().now()
                elif response[0] == STM32Response.RESP_ERROR.value:
                    self.handle_error(response[1] if len(response) > 1 else 0)
        
        # 发布关节状态
        self.publish_joint_state()
        
        # 发布机械臂状态
        self.publish_arm_status()
    
        # 处理错误状态
        if self.error_code != STM32ErrorCode.ERR_NONE.value:
            self.handle_error(self.error_code)
            self.error_code = STM32ErrorCode.ERR_NONE.value  # 重置错误码

    def emergency_stop(self):
        """发送紧急停止命令"""
        self.get_logger().error("EMERGENCY STOP COMMAND SENT")
        self.send_command(STM32Command.CMD_EMERGENCY_STOP.value, b'')
        self.trajectory_active = False

    def home_arm(self):
        """发送回零命令"""
        self.get_logger().info("Sending home command")
        return self.send_command_with_retry(
            STM32Command.CMD_HOME.value,
            b'',
            expected_response=STM32Response.RESP_ACK.value
        )

    def destroy_node(self):
        """节点销毁时关闭I2C连接"""
        if self.i2c_bus:
            try:
                self.i2c_bus.close()
            except:
                pass
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