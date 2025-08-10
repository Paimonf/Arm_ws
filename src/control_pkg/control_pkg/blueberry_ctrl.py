import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from blueberry_interfaces.msg import Berry, DetectedBerries
from geometry_msgs.msg import Point
import numpy as np
import math

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        
        # 参数配置
        self.declare_parameters(
            namespace='',
            parameters=[
                ('arm_joints', ['base_rotation_joint', 'joint1', 'joint2', 'joint3']),
                ('gripper_joints', ['gripper_joint']),
                ('approach_distance', 0.05),  # 接近蓝莓的安全距离
                ('gripper_open', 0.04),      # 夹爪打开宽度
                ('gripper_close', 0.01),     # 夹爪闭合宽度
                ('home_position', [0.0, 0.0, 0.0, 0.0]),  # 初始位置
                ('joint_limits', {
                    'base_rotation_joint': [-3.14, 3.14],
                    'joint1': [-1.57, 1.57],
                    'joint2': [-1.57, 1.57],
                    'joint3': [-1.57, 1.57]
                }),
                ('ik_params', {
                    'base_height': 0.123,  # 基座高度
                    'link1_length': 0.129,  # 第一臂长度
                    'link2_length': 0.129,  # 第二臂长度
                    'link3_length': 0.1     # 第三臂长度
                })
            ]
        )
        
        # 初始化变量
        self.current_joint_positions = {}
        self.current_berries = []
        self.is_busy = False
        
        # 创建动作客户端
        self.arm_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )
        
        # 订阅蓝莓检测结果
        self.berry_sub = self.create_subscription(
            DetectedBerries,
            '/detected_berries',
            self.berry_callback,
            10
        )
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info("Arm Control Node initialized")
        self.send_to_home_position()

    def joint_state_callback(self, msg):
        """更新当前关节位置"""
        for i, name in enumerate(msg.name):
            self.current_joint_positions[name] = msg.position[i]

    def berry_callback(self, msg):
        """处理检测到的蓝莓位置"""
        if self.is_busy:
            self.get_logger().info("Arm is busy, ignoring new berries")
            return
            
        self.current_berries = msg.berries
        self.process_next_berry()

    def process_next_berry(self):
        """处理下一个蓝莓采摘"""
        if not self.current_berries:
            self.get_logger().info("No berries to process")
            return
            
        self.is_busy = True
        berry = self.current_berries.pop(0)
        
        # 执行采摘序列
        self.pick_berry(berry)

    def pick_berry(self, berry):
        """执行完整的采摘动作序列"""
        self.get_logger().info(f"Picking berry at ({berry.position.x:.3f}, {berry.position.y:.3f}, {berry.position.z:.3f})")
        
        # 步骤1: 移动到接近位置
        approach_pos = self.calculate_approach_position(berry.position)
        if not self.move_arm_to_position(approach_pos):
            self.get_logger().error("Failed to move to approach position")
            self.is_busy = False
            return
        
        # 步骤2: 打开夹爪
        self.control_gripper(self.get_parameter('gripper_open').value)
        
        # 步骤3: 移动到采摘位置
        if not self.move_arm_to_position(berry.position):
            self.get_logger().error("Failed to move to harvest position")
            self.is_busy = False
            return
        
        # 步骤4: 闭合夹爪
        self.control_gripper(self.get_parameter('gripper_close').value)
        
        # 步骤5: 抬起机械臂
        lift_pos = Point(
            x=berry.position.x,
            y=berry.position.y,
            z=berry.position.z + 0.1
        )
        if not self.move_arm_to_position(lift_pos):
            self.get_logger().error("Failed to lift arm")
            self.is_busy = False
            return
        
        # 步骤6: 返回归位位置
        self.send_to_home_position()
        
        # 步骤7: 释放夹爪
        self.control_gripper(self.get_parameter('gripper_open').value)
        
        self.is_busy = False
        self.process_next_berry()  # 处理下一个蓝莓

    def calculate_approach_position(self, target_pos):
        """计算接近位置 (保持安全距离)"""
        approach_dist = self.get_parameter('approach_distance').value
        direction = np.array([target_pos.x, target_pos.y, target_pos.z])
        norm = np.linalg.norm(direction)
        
        if norm > 0:
            direction = direction / norm
            approach_vec = direction * approach_dist
            return Point(
                x=target_pos.x - approach_vec[0],
                y=target_pos.y - approach_vec[1],
                z=target_pos.z - approach_vec[2]
            )
        return target_pos

    def send_to_home_position(self):
        """发送机械臂到归位位置"""
        home_pos = self.get_parameter('home_position').value
        self.move_arm_to_joints(home_pos)

    def move_arm_to_position(self, position):
        """移动机械臂到笛卡尔空间位置"""
        joint_positions = self.inverse_kinematics(position)
        if joint_positions:
            return self.move_arm_to_joints(joint_positions)
        return False

    def inverse_kinematics(self, target_position):
        """根据机械臂URDF结构实现的逆运动学算法"""
        # 获取参数
        params = self.get_parameter('ik_params').value
        BASE_HEIGHT = params['base_height']
        LINK1_LENGTH = params['link1_length']
        LINK2_LENGTH = params['link2_length']
        LINK3_LENGTH = params['link3_length']
        joint_limits = self.get_parameter('joint_limits').value
        
        try:
            # 1. 计算底座旋转角度
            base_angle = math.atan2(target_position.y, target_position.x)
            
            # 检查关节限位
            base_limits = joint_limits.get('base_rotation_joint', [-3.14, 3.14])
            if base_angle < base_limits[0] or base_angle > base_limits[1]:
                self.get_logger().warn(f"Base angle {base_angle:.2f} exceeds limits")
                base_angle = max(min(base_angle, base_limits[1]), base_limits[0])
            
            # 2. 计算平面内的目标位置 (投影到机械臂平面)
            planar_distance = math.sqrt(target_position.x**2 + target_position.y**2)
            planar_height = target_position.z - BASE_HEIGHT
            
            # 3. 计算关节1、2、3的角度
            # 目标点到第一关节的距离
            d = math.sqrt(planar_distance**2 + planar_height**2)
            
            # 检查是否可达
            max_reach = LINK1_LENGTH + LINK2_LENGTH + LINK3_LENGTH
            if d > max_reach:
                self.get_logger().error(f"Target position is out of reach (d={d:.3f}, max={max_reach:.3f})")
                return None
            
            # 使用余弦定理计算角度
            # 计算关节1角度
            alpha = math.atan2(planar_height, planar_distance)
            try:
                beta = math.acos(
                    (LINK1_LENGTH**2 + d**2 - LINK2_LENGTH**2) / 
                    (2 * LINK1_LENGTH * d)
                )
            except ValueError:
                # 处理数学域错误
                beta = 0
                
            theta1 = alpha + beta
            
            # 计算关节2角度
            try:
                gamma = math.acos(
                    (LINK1_LENGTH**2 + LINK2_LENGTH**2 - d**2) / 
                    (2 * LINK1_LENGTH * LINK2_LENGTH)
                )
            except ValueError:
                gamma = 0
                
            theta2 = math.pi - gamma
            
            # 计算关节3角度 (保持末端水平)
            theta3 = - (theta1 + theta2)
            
            # 4. 应用关节限位
            joint1_limits = joint_limits.get('joint1', [-1.57, 1.57])
            joint2_limits = joint_limits.get('joint2', [-1.57, 1.57])
            joint3_limits = joint_limits.get('joint3', [-1.57, 1.57])
            
            theta1 = max(min(theta1, joint1_limits[1]), joint1_limits[0])
            theta2 = max(min(theta2, joint2_limits[1]), joint2_limits[0])
            theta3 = max(min(theta3, joint3_limits[1]), joint3_limits[0])
            
            self.get_logger().info(
                f"IK solution: base={base_angle:.2f}, "
                f"joint1={theta1:.2f}, joint2={theta2:.2f}, joint3={theta3:.2f}"
            )
            
            return [base_angle, theta1, theta2, theta3]
        
        except Exception as e:
            self.get_logger().error(f"IK calculation failed: {str(e)}")
            return None

    def move_arm_to_joints(self, positions):
        """发送关节轨迹到机械臂控制器"""
        joint_names = self.get_parameter('arm_joints').value
        
        if len(joint_names) != len(positions):
            self.get_logger().error("Joint positions mismatch")
            return False
            
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        # 创建平滑轨迹 (3个点)
        point1 = JointTrajectoryPoint()
        point1.positions = self.get_current_joint_values(joint_names)
        point1.time_from_start = rclpy.time.Duration(seconds=0.0).to_msg()
        
        point2 = JointTrajectoryPoint()
        point2.positions = [
            (current + target) * 0.5 
            for current, target in zip(point1.positions, positions)
        ]
        point2.time_from_start = rclpy.time.Duration(seconds=1.0).to_msg()
        
        point3 = JointTrajectoryPoint()
        point3.positions = positions
        point3.time_from_start = rclpy.time.Duration(seconds=2.0).to_msg()
        
        trajectory.points = [point1, point2, point3]
        goal_msg.trajectory = trajectory
        
        # 发送目标并等待结果
        self.arm_action_client.wait_for_server()
        future = self.arm_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.arm_goal_response_callback)
        
        return True

    def get_current_joint_values(self, joint_names):
        """获取当前关节位置"""
        return [self.current_joint_positions.get(name, 0.0) for name in joint_names]

    def arm_goal_response_callback(self, future):
        """处理机械臂动作响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected')
            return
        
        self.get_logger().info('Arm goal accepted')
        future = goal_handle.get_result_async()
        future.add_done_callback(self.arm_result_callback)

    def arm_result_callback(self, future):
        """处理机械臂动作结果"""
        result = future.result().result
        if result.error_code == result.SUCCESSFUL:
            self.get_logger().info('Arm movement succeeded')
        else:
            self.get_logger().error(f'Arm movement failed: {result.error_string}')

    def control_gripper(self, width):
        """控制夹爪开合"""
        joint_names = self.get_parameter('gripper_joints').value
        
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [width]
        point.time_from_start = rclpy.time.Duration(seconds=1.0).to_msg()
        
        trajectory.points = [point]
        goal_msg.trajectory = trajectory
        
        self.gripper_action_client.wait_for_server()
        self.gripper_action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()