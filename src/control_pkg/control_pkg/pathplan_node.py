#!/usr/bin/env python3
import rclpy 
from rclpy.node  import Node 
from rclpy.action  import ActionClient
from rclpy.duration  import Duration 
 
import numpy as np
import math
 
from blueberry_interfaces.srv  import DetectBerries, PathPlan
from blueberry_interfaces.msg  import Berry, DetectedBerries 
from geometry_msgs.msg  import Point, PoseStamped
from trajectory_msgs.msg  import JointTrajectory, JointTrajectoryPoint
from control_msgs.action  import FollowJointTrajectory
from sensor_msgs.msg  import JointState  # 新增：导入关节状态消息 
 
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
                ('arm.base_height',  0.103),  # 基座到关节1的高度
                ('home_position', [0.0, 0.5, 0.5, -1.0]),  # 起始位置 [base_rotation, joint1, joint2, joint3]
                ('approach_distance', 0.05),  # 采摘时末端执行器到蓝莓的距离 
                ('harvest_time', 2.0),  # 采摘一个蓝莓所需时间(秒)
                ('move_time', 3.0),     # 移动位置所需时间(秒)
                ('joint_state_topic', '/joint_states'),  # 新增：关节状态话题 
                ('position_tolerance', 0.05),  # 新增：关节位置容差 
                ('velocity_tolerance', 0.01),  # 新增：关节速度容差 
            ]
        )
        
        # 获取参数
        self.l1 = self.get_parameter('arm.l1').value 
        self.l2 = self.get_parameter('arm.l2').value 
        self.l3 = self.get_parameter('arm.l3').value 
        self.base_height  = self.get_parameter('arm.base_height').value 
        self.home_position  = self.get_parameter('home_position').value  
        self.approach_distance  = self.get_parameter('approach_distance').value 
        self.harvest_time  = self.get_parameter('harvest_time').value 
        self.move_time  = self.get_parameter('move_time').value  
        self.joint_state_topic  = self.get_parameter('joint_state_topic').value  
        self.position_tolerance  = self.get_parameter('position_tolerance').value 
        self.velocity_tolerance  = self.get_parameter('velocity_tolerance').value 
        
        # 新增：当前关节状态存储 
        self.current_joint_positions  = None 
        self.current_joint_velocities  = None 
        self.joint_state_received  = False 
        
        # 创建检测服务客户端 
        self.detect_client  = self.create_client(DetectBerries,  'detect_berries')
        
        # 创建采摘服务
        self.harvest_service  = self.create_service( 
            PathPlan, 
            'harvest_berries', 
            self.harvest_callback  
        )
        
        # 创建关节轨迹动作客户端 
        self.joint_trajectory_client  = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # 新增：订阅关节状态话题
        self.joint_state_sub  = self.create_subscription( 
            JointState,
            self.joint_state_topic, 
            self.joint_state_callback, 
            10 
        )
        self.get_logger().info(f"Subscribed  to joint state topic: {self.joint_state_topic}") 
        
        # 等待动作服务器可用 
        self.get_logger().info("Waiting  for joint trajectory action server...")
        self.joint_trajectory_client.wait_for_server() 
        self.get_logger().info("Joint  trajectory action server available")
        
        # 关节名称顺序 (必须与URDF一致)
        self.joint_names  = [
            'base_rotation_joint',
            'joint1',
            'joint2',
            'joint3'
        ]
        self.joint_limits = {
            'base_rotation': (-math.pi, math.pi),  # 基座旋转范围
            'joint1': (0, math.pi/2),             # 关节1范围
            'joint2': (0, math.pi/2),             # 关节2范围
            'joint3': (-math.pi, 0)               # 关节3范围
        }
        # 新增：等待获取初始关节状态
        self.wait_for_initial_joint_state() 
        
        self.get_logger().info("Berry  Harvesting Node initialized")
 
    def wait_for_initial_joint_state(self):
        """等待获取初始关节状态"""
        self.get_logger().info("Waiting  for initial joint state...")
        while not self.joint_state_received  and rclpy.ok(): 
            rclpy.spin_once(self,  timeout_sec=0.1)
        self.get_logger().info("Initial  joint state received")
 
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
        self.current_joint_positions  = [
            current_positions.get(name,  0.0) for name in self.joint_names  
        ]
        
        self.current_joint_velocities  = [
            current_velocities.get(name,  0.0) for name in self.joint_names 
        ]
        
        # 标记已收到关节状态
        self.joint_state_received  = True
        self.get_logger().debug(f"Updated  joint positions: {self.current_joint_positions}") 
 
    def check_joint_state(self):
        """检查关节状态是否准备好"""
        if not self.joint_state_received: 
            self.get_logger().error("Joint  state not yet received")
            return False
        
        # 检查关节速度是否接近零（安全移动条件）
        if any(abs(v) > self.velocity_tolerance  for v in self.current_joint_velocities): 
            self.get_logger().warn("Joints  are still moving, waiting for stabilization")
            return False 
            
        return True
 
    def harvest_callback(self, request, response):
        """处理采摘服务请求"""
        if not request.start: 
            response.success  = False
            response.message  = "Harvesting not started"
            return response 
        
        # 检查关节状态是否准备好 
        if not self.check_joint_state(): 
            response.success  = False
            response.message  = "Joints not in stable state"
            return response
        
        detect_req = DetectBerries.Request()
        detect_req.start  = True

        # 调用检测服务获取蓝莓位置 
        self.get_logger().info("Requesting  berry detection...")
        detect_future = self.detect_client.call_async(detect_req) 
        detect_future.add_done_callback(self.detection_done_callback) 
        
        response.success  = True 
        response.message  = "Harvesting sequence started"
        return response
 
    def detection_done_callback(self, future):
        """处理检测服务响应"""
        try:
            response = future.result() 
            if not response.berries.berries: 
                self.get_logger().warn("No  berries detected")
                return
                
            berries = response.berries.berries 
            self.get_logger().info(f"Detected  {len(berries)} ripe berries")
            
            # 规划并执行采摘路径 
            self.plan_harvest_path(berries) 
            
        except Exception as e:
            self.get_logger().error(f"Detection  service call failed: {str(e)}")
 
    def plan_harvest_path(self, berries):
        """规划采摘路径，考虑当前关节状态"""
        # 创建轨迹消息 
        trajectory = JointTrajectory()
        trajectory.joint_names  = self.joint_names 
        
        # 时间从零开始
        time_from_start = 0.0
        
        # 第一步: 从当前位置开始 
        start_point = self.create_trajectory_point( 
            self.current_joint_positions,   # 使用当前关节位置 
            time_from_start
        )
        trajectory.points.append(start_point) 
        
        # 第二步: 移动到起始位置（如果不在起始位置）
        if not self.is_at_position(self.home_position,  self.position_tolerance): 
            time_from_start += self.move_time 
            home_point = self.create_trajectory_point( 
                self.home_position,  
                time_from_start 
            )
            trajectory.points.append(home_point) 
            self.get_logger().info("Adding  move to home position")
        else:
            self.get_logger().info("Already  at home position")
        
        # 对每个蓝莓进行采摘规划 
        for i, berry in enumerate(berries):
            self.get_logger().info(f"Planning  path for berry {i+1} at position: "
                                  f"x={berry.position.x:.3f},  y={berry.position.y:.3f},  z={berry.position.z:.3f}") 
            
            # 计算采摘该蓝莓所需的关节角度
            joint_angles = self.calculate_joint_angles(berry.position) 
            
            if joint_angles is None:
                self.get_logger().warn(f"Skipping  berry {i+1} - unreachable")
                continue
                
            # 添加移动到位姿点的轨迹点
            time_from_start += self.move_time  
            move_point = self.create_trajectory_point( 
                joint_angles,
                time_from_start 
            )
            trajectory.points.append(move_point) 
            
            # 模拟采摘动作 (保持位置)
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
        goal_msg.trajectory  = trajectory
        
        self.get_logger().info("Sending  trajectory to joint controller...")
        self.send_goal_future  = self.joint_trajectory_client.send_goal_async( 
            goal_msg,
            feedback_callback=self.feedback_callback 
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback) 
 
    def is_at_position(self, target_positions, tolerance):
        """检查当前关节是否在目标位置附近"""
        if not self.joint_state_received: 
            return False 
            
        for current, target in zip(self.current_joint_positions,  target_positions):
            if abs(current - target) > tolerance:
                return False 
        return True 
 
    def feedback_callback(self, feedback_msg):
        """处理动作反馈"""
        feedback = feedback_msg.feedback  
        self.get_logger().debug( 
            f"Trajectory execution: {feedback.actual.time_from_start.sec}." 
            f"{feedback.actual.time_from_start.nanose  * 1e-9:.2f}s elapsed"
        )
 
    def goal_response_callback(self, future):
        """处理动作目标响应"""
        goal_handle = future.result() 
        if not goal_handle.accepted: 
            self.get_logger().error("Trajectory  rejected by controller")
            return 
            
        self.get_logger().info("Trajectory  accepted, executing...")
        self.result_future  = goal_handle.get_result_async() 
        self.result_future.add_done_callback(self.get_result_callback) 
 
    def get_result_callback(self, future):
        """处理动作结果"""
        result = future.result().result 
        if result.error_code  == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trajectory  executed successfully")
            # 更新当前关节状态为最后位置（通常是home位置）
            if self.trajectory.points: 
                self.current_joint_positions  = self.trajectory.points[-1].positions 
        else:
            self.get_logger().error(f"Trajectory  execution failed: error code {result.error_code}") 
 
    def create_trajectory_point(self, positions, time_sec):
        """创建轨迹点"""
        point = JointTrajectoryPoint()
        point.positions  = positions
        point.time_from_start  = Duration(seconds=time_sec).to_msg()
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
        # 末端执行器方向：垂直向下（φ = π）
        phi = math.pi
        
        # 4. 调用逆运动学函数
        angles = self.inverse_kinematics(
            x=target_x,
            y=target_z,  # 注意：这里是target_z（高度）
            phi=phi,
            L1=self.l1,
            L2=self.l2,
            L3=self.l3
        )
        
        if angles is None:
            self.get_logger().warn("Target position unreachable by inverse kinematics")
            return None
            
        # 解包角度：θ₁, θ₂, θ₃
        theta1, theta2, theta3 = angles
        
        # 5. 组合所有关节角度 [base_rotation, θ₁, θ₂, θ₃]
        joint_angles = [base_angle, theta1, theta2, theta3]
        
        # 6. 检查关节角度是否在限位范围内
        joint_names = ['base_rotation', 'joint1', 'joint2', 'joint3']
        
        for i, (angle, name) in enumerate(zip(joint_angles, joint_names)):
            lower, upper = self.joint_limits[name]
            if not (lower <= angle <= upper):
                self.get_logger().warn(
                    f"Joint {name} angle {math.degrees(angle):.1f}° "
                    f"out of range [{math.degrees(lower):.1f}°, {math.degrees(upper):.1f}°]"
                )
                return None 
                
        return joint_angles

    # 添加逆运动学函数（确保在类外部定义）
    def inverse_kinematics(self,x, y, phi, L1, L2, L3):
        # 步骤1：计算腕关节位置
        x3 = x - L3 * math.sin(phi)
        y3 = y - L3 * math.cos(phi)
        
        # 步骤2：检查可达性
        d = math.sqrt(x3**2 + y3**2)
        r_min = math.sqrt(L1**2 + L2**2)
        r_max = L1 + L2
        
        if not (r_min <= d <= r_max) or x3 < 0 or y3 < 0:
            return None  # 目标不可达
        
        # 步骤3：计算θ₂
        cos_theta2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2 = math.acos(cos_theta2)  # 唯一解
        
        # 步骤4：计算θ₁
        beta = math.atan2(y3, x3)  # 注意参数顺序(y,x)
        cos_alpha = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
        alpha = math.acos(cos_alpha)
        theta1 = math.pi/2 - (beta + alpha)
        
        # 步骤5：计算θ₃
        theta3 = phi - (theta1 + theta2)
        
        # 步骤6：验证约束
        angles = [theta1, theta2, theta3]
        if all(0 <= angle <= math.pi/2 for angle in angles):
            return theta1, theta2, theta3
        else:
            return None  # 角度越界
 
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