#!/usr/bin/env python3
# pathplan_node.py
import rclpy 
from rclpy.node import Node 
from rclpy.action import ActionClient
from rclpy.duration import Duration 
import numpy as np
import math
from enum import Enum
 
from blueberry_interfaces.srv import PathPlan
from blueberry_interfaces.msg import Berry, DetectedBerries 
from geometry_msgs.msg import Point, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

class BerryHarvestingNode(Node):
    def __init__(self):
        super().__init__('berry_harvesting_node')
        
        # 机械臂参数 (单位: 米)
        self.declare_parameters( 
            namespace='',
            parameters=[
                ('arm.l1', 0.129),   # 关节1到关节2的长度
                ('arm.l2', 0.129),   # 关节2到关节3的长度
                ('arm.l3', 0.121),   # 关节3到末端执行器的长度 
                ('arm.base_height', 0.103),  # 基座到关节1的高度
                ('home_position', [0.0, 0.0, 0.0, 2.0]),  # 起始位置 [base_rotation, joint1, joint2, joint3]
                ('approach_distance', 0.05),  # 采摘时末端执行器到蓝莓的距离 
                ('harvest_time', 2.0),  # 采摘一个蓝莓所需时间(秒)
                ('move_time', 3.0),     # 移动位置所需时间(秒)
                ('joint_state_topic', '/joint_states'),  # 关节状态话题 
                ('position_tolerance', 0.05),  # 关节位置容差 
                ('velocity_tolerance', 0.01),  # 关节速度容差 
                ('ik_solution_attempts', 10),  # 逆运动学求解尝试次数
                ('preferred_orientation', 1.57),  # 首选末端方向（弧度），0表示垂直向下
                ('PWM_limits',[50,80])
            ]
        )
        
        # 获取参数
        self.l1 = self.get_parameter('arm.l1').value 
        self.l2 = self.get_parameter('arm.l2').value 
        self.l3 = self.get_parameter('arm.l3').value 
        self.base_height = self.get_parameter('arm.base_height').value 
        self.home_position = self.get_parameter('home_position').value  
        self.approach_distance = self.get_parameter('approach_distance').value 
        self.harvest_time = self.get_parameter('harvest_time').value 
        self.move_time = self.get_parameter('move_time').value  
        self.joint_state_topic = self.get_parameter('joint_state_topic').value  
        self.position_tolerance = self.get_parameter('position_tolerance').value 
        self.velocity_tolerance = self.get_parameter('velocity_tolerance').value 
        self.ik_solution_attempts = self.get_parameter('ik_solution_attempts').value
        self.preferred_orientation = self.get_parameter('preferred_orientation').value
        self.PWM_limits=self.get_parameter('PWM_limits').value
        self.min_PWM = self.PWM_limits[0]
        self.max_PWM = self.PWM_limits[1]
        
        # 当前关节状态存储 
        self.current_joint_positions = None 
        self.current_joint_velocities = None 
        self.joint_state_received = False 
        
        # 创建路径规划服务
        self.path_plan_service = self.create_service(
            PathPlan,
            'path_plan',
            self.path_plan_callback
        )
        
        # 创建关节轨迹动作客户端 
        self.joint_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # 订阅关节状态话题
        self.joint_state_sub = self.create_subscription( 
            JointState,
            self.joint_state_topic, 
            self.joint_state_callback, 
            10 
        )
        self.get_logger().info(f"Subscribed to joint state topic: {self.joint_state_topic}") 
        
        # 等待动作服务器可用 
        self.get_logger().info("Waiting for joint trajectory action server...")
        self.joint_trajectory_client.wait_for_server() 
        self.get_logger().info("Joint trajectory action server available")
        
        # 关节名称顺序 (必须与URDF一致)
        self.joint_names = [
            'base_rotation_joint',
            'joint1',
            'joint2',
            'joint3'
        ]
        
        # 关节限位
        self.joint_limits = {
            'base_rotation_joint': (-math.pi*2/3, math.pi*2/3),  # 基座旋转范围
            'joint1': (-math.pi*2/3, math.pi*2/3),             # 关节1范围
            'joint2': (-math.pi*2/3, math.pi*2/3),             # 关节2范围
            'joint3': (-math.pi*2/3, math.pi*2/3)               # 关节3范围
        }
        
        # 等待获取初始关节状态
        self.wait_for_initial_joint_state() 
        
        self.get_logger().info("Path Plan Service initialized")
 
    def wait_for_initial_joint_state(self):
        """等待获取初始关节状态"""
        self.get_logger().info("Waiting for initial joint state...")
        while not self.joint_state_received and rclpy.ok(): 
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Initial joint state received")
 
    def joint_state_callback(self, msg):
        """处理关节状态更新"""
        # 提取我们关心的关节状态 
        current_positions = {}
        current_velocities = {}
        
        for i, name in enumerate(msg.name): 
            if name in self.joint_names: 
                current_positions[name] = msg.position[i] 
                if i < len(msg.velocity): 
                    current_velocities[name] = msg.velocity[i] 
                else:
                    current_velocities[name] = 0.0
        
        # 按joint_names顺序存储位置和速度
        self.current_joint_positions = [
            current_positions.get(name, 0.0) for name in self.joint_names  
        ]
        
        self.current_joint_velocities = [
            current_velocities.get(name, 0.0) for name in self.joint_names 
        ]
        
        # 标记已收到关节状态
        self.joint_state_received = True
 
    def path_plan_callback(self, request, response):
        """处理路径规划服务请求"""
        # 检查关节状态是否准备好 
        if not self.check_joint_state(): 
            response.success = False
            response.message = "Joints not in stable state"
            return response
        
        # 规划并执行采摘路径
        success = self.plan_and_execute_path(request.berries.berries)
        
        if success:
            response.success = True
            response.message = "Path planning and execution successful"
        else:
            response.success = False
            response.message = "Path planning or execution failed"
            
        return response
 
    def plan_and_execute_path(self, berries):
        """规划并执行采摘路径"""
        self.get_logger().info(f"Planning path for {len(berries)} berries" ) 
        
        # 计算采摘该蓝莓所需的关节角度
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names 
        
        # 时间从零开始
        time_from_start = 0.0
        
        # 第一步: 从当前位置开始 
        start_point = self.create_trajectory_point( 
            self.current_joint_positions,  # 使用当前关节位置 
            time_from_start
        )
        trajectory.points.append(start_point) 
        
        # 第二步: 移动到起始位置（如果不在起始位置）
        # if not self.is_at_position(self.home_position, self.position_tolerance): 
        #     time_from_start += self.move_time 
        home_point = self.create_trajectory_point( 
            self.home_position,  
            time_from_start 
        )
        trajectory.points.append(home_point) 
        self.get_logger().info("Adding move to home position")
        # else:
        #     self.get_logger().info("Already at home position")
        
        # 对每个蓝莓进行采摘规划 
        for i, berry in enumerate(berries):
            self.get_logger().info(f"Planning path for berry {i+1} at position:"
                                  f"x={berry.position.x:.3f}, y={berry.position.y:.3f}, z={berry.position.z:.3f}"
                                  f"size={berry.size}") 
            # 计算采摘该蓝莓所需的关节角度
            joint_angles = self.calculate_joint_angles(berry.position) 
            
            if joint_angles is None:
                self.get_logger().warn(f"跳过蓝莓 {i+1} - unreachable")
                continue
                
            # 添加移动到位姿点的轨迹点
            time_from_start += self.move_time  
            move_point = self.create_trajectory_point( 
                joint_angles,
                time_from_start 
            )
            trajectory.points.append(move_point) 
            
            # 模拟采摘动作 (保持位置)
            joint_angles[3]=berry.size
            time_from_start += self.harvest_time 
            harvest_point = self.create_trajectory_point( 
                joint_angles,
                time_from_start
            )
            trajectory.points.append(harvest_point) 
            
        # 返回到安全位置 (起始位置)
        time_from_start += self.move_time  
        safe_point = self.create_trajectory_point( 
            self.home_position, 
            time_from_start 
        )
        trajectory.points.append(safe_point) 
        
        # 创建并发送动作目标
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        self.get_logger().info("Sending trajectory to joint controller...")
        
        self.send_goal_future = self.joint_trajectory_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback 
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        
        
        return True
 
    def check_joint_state(self):
        """检查关节状态是否准备好"""
        if not self.joint_state_received: 
            self.get_logger().error("Joint state not yet received")
            return False
        
        # 检查关节速度是否接近零（安全移动条件）
        if any(abs(v) > self.velocity_tolerance for v in self.current_joint_velocities): 
            self.get_logger().warn("Joints are still moving, waiting for stabilization")
            return False 
            
        return True
 
    def is_at_position(self, target_positions, tolerance):
        """检查当前关节是否在目标位置附近"""
        if not self.joint_state_received: 
            self.get_logger().info("没有接受到关节状态信息! /is_at_position() 函数将返回 False.")
            return False 
            
        for current, target in zip(self.current_joint_positions, target_positions):
            if abs(current - target) > tolerance:
                return False 
        return True 
 
    def feedback_callback(self, feedback_msg):
        """处理动作反馈"""
        feedback = feedback_msg.feedback  
        self.get_logger().debug( 
            f"Trajectory execution: {feedback.actual.time_from_start.sec}." 
            f"{feedback.actual.time_from_start.nanosec * 1e-9:.2f}s elapsed"
        )
 
    def goal_response_callback(self, future):
        """处理动作目标响应"""
        goal_handle = future.result() 
        if not goal_handle.accepted: 
            self.get_logger().error("Trajectory rejected by controller")
            return 
            
        self.get_logger().info("Trajectory accepted, executing...")
        self.result_future = goal_handle.get_result_async() 
        self.result_future.add_done_callback(self.get_result_callback) 
 
    def get_result_callback(self, future):
        """处理动作结果"""
        result = future.result().result 
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trajectory executed successfully")
            # 更新当前关节状态为最后位置（通常是home位置）
            self.current_joint_positions = self.home_position
        else:
            self.get_logger().error(f"Trajectory execution failed: error code {result.error_code}") 
 

    def create_trajectory_point(self, positions, time_sec,):
        """创建轨迹点"""
        point = JointTrajectoryPoint()
        self.get_logger().info(f"位置：{positions}")
        point.positions = positions
        point.time_from_start = Duration(seconds=time_sec).to_msg()
        #point.effort=
        return point 
 
    def calculate_joint_angles(self, berry_position):
        """
        计算采摘蓝莓所需的关节角度
        
        参数:
            berry_position: 蓝莓的位置 (相对于base_link坐标系)
            
        返回:
            list: [base_angle, q1, q2, q3] 关节角度 (弧度)
        """
        # 1. 计算基座旋转角度
        base_angle = math.atan2(berry_position.y, berry_position.x)
        
        # 2. 将目标点转换到机械臂平面
        planar_distance = math.sqrt(berry_position.x**2 + berry_position.y**2)
        target_x = planar_distance
        target_z = berry_position.z - self.base_height 
        
        # 3. 调整目标点：末端执行器在蓝莓上方approach_distance处
        # 现在不固定末端方向，而是尝试多种可能的末端方向
        
        best_solution = None
        best_orientation_diff = float('inf')
        
        # 尝试多种末端方向，找到最优解
        for attempt in range(self.ik_solution_attempts):
            # 生成不同的末端方向尝试值
            # 从首选方向开始，然后在两侧尝试不同的角度
            angle_variation = (attempt // 2) * 0.2 * (-1 if attempt % 2 == 0 else 1)
            phi = self.preferred_orientation + angle_variation
            
            # 调用逆运动学函数
            angles = self.inverse_kinematics(
                x=target_x,
                z=target_z,  # 注意：这里是target_z（高度）
                phi=phi,
                L1=self.l1,
                L2=self.l2,
                L3=self.l3
            )
            
            if angles is None:
                continue
                
            # 解包角度：θ₁, θ₂, θ₃
            theta1, theta2, theta3 = angles
            
            # 组合所有关节角度 [base_rotation, θ₁, θ₂, θ₃]
            joint_angles = [base_angle, theta1, theta2, theta3]
            
            # 检查关节角度是否在限位范围内
            if not self.check_joint_limits(joint_angles):
                continue
                
            # 评估这个解的优劣（这里简单使用与首选方向的差异）
            orientation_diff = abs(phi - self.preferred_orientation)
            
            if orientation_diff < best_orientation_diff:
                best_orientation_diff = orientation_diff
                best_solution = joint_angles
        
        if best_solution is None:
            self.get_logger().warn("无法找到合适的关节角度解!")
            return None
            
        self.get_logger().info(f'找到最优角度解: {best_solution}')
        return best_solution

    def check_joint_limits(self, joint_angles):
        """检查关节角度是否在限位范围内"""
        for i, (angle, name) in enumerate(zip(joint_angles, self.joint_names)):
            lower, upper = self.joint_limits[name]
            if not (lower <= angle <= upper):
                return False
        return True

    def size_to_PWM(self,size):
        return ((size-15)*2+50)

    def inverse_kinematics(self, x, z, phi, L1, L2, L3):
        """
        改进的逆运动学计算，允许末端执行器角度自由变化
        
        参数:
            x: 目标点的x坐标（机械臂平面内）
            z: 目标点的z坐标（高度）
            phi: 末端执行器相对于水平方向的角度
            L1, L2, L3: 机械臂各段长度
            
        返回:
            tuple: (theta1, theta2, theta3) 或 None（如果无解）
        """
        try:
            # 计算腕部位置
            wrist_x = x - L3 * math.sin(phi)
            wrist_z = z - L3 * math.cos(phi)
            
            # 计算腕部到原点的距离
            d = math.sqrt(wrist_x**2 + wrist_z**2)
            
            # 检查是否可达
            if d > (L1 + L2) or d < abs(L1 - L2):
                self.get_logger().warn("目标位置无法到达")
                return None
                
            # 计算theta2（肘部角度）
            cos_theta2 = (wrist_x**2 + wrist_z**2 - L1**2 - L2**2) / (2 * L1 * L2)
            # cos_theta2 = max(min(cos_theta2, 1.0), -1.0)  # 限制在[-1, 1]范围内
            theta2 = math.acos(cos_theta2)
            
            # 计算theta1（肩部角度）
            alpha = math.atan2(wrist_z, wrist_x)
            beta = math.acos((L1**2 + d**2 - L2**2) / (2 * L1 * d))
            theta1 = math.pi/2 - alpha - beta
            
            # 计算theta3（腕部角度）
            theta3 = phi - theta1 - theta2
            
            # 返回角度
            return theta1, theta2, theta3
            
        except Exception as e:
            self.get_logger().warn(f"逆运动学计算错误: {str(e)}")
            return None
 
def main(args=None):
    rclpy.init(args=args) 
    node = BerryHarvestingNode()
    
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node() 
        rclpy.shutdown() 
 
if __name__ == '__main__':
    main()