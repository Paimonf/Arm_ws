#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import numpy as np
import math

from blueberry_interfaces.srv import DetectBerries, PathPlan
from blueberry_interfaces.msg import Berry, DetectedBerries
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState   # 仅用于发布目标关节角

class BerryHarvestingNode(Node):
    def __init__(self):
        super().__init__('berry_harvesting_node')

        # ----------------- 参数 -----------------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('arm.l1', 0.129),
                ('arm.l2', 0.129),
                ('arm.l3', 0.121),
                ('arm.base_height', 0.103),
                ('home_position', [0.0, 0.0, 0.0, 2.0]),
                ('approach_distance', 0.05),
                ('harvest_time', 2.0),
                ('move_time', 3.0),
                ('target_joint_topic', '/target_joint_states'),
            ]
        )

        self.l1 = self.get_parameter('arm.l1').value
        self.l2 = self.get_parameter('arm.l2').value
        self.l3 = self.get_parameter('arm.l3').value
        self.base_height = self.get_parameter('arm.base_height').value
        self.home_position = self.get_parameter('home_position').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.target_joint_topic = self.get_parameter('target_joint_topic').value

        # ----------------- 客户端 / 服务 / 发布者 -----------------
        self.detect_client = self.create_client(DetectBerries, 'detect_berries')

        self.harvest_service = self.create_service(
            PathPlan,
            'harvest_berries',
            self.harvest_callback
        )

        # 发布目标关节角
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_command',
            QoSProfile(depth=10)
        )

        # 关节名称（与 URDF 保持一致）
        self.joint_names = [
            'base_rotation_joint',
            'joint1',
            'joint2',
            'joint3'
        ]
        self.joint_limits = {
            'base_rotation': (-math.pi, math.pi),
            'joint1': (-math.pi/2, math.pi/2),
            'joint2': (0, math.pi),
            'joint3': (-math.pi/2, math.pi)
        }

        self.get_logger().info('Berry Harvesting Node (joint angle publisher) initialized')

    # ------------------------------------------------------------------
    # 业务主流程
    # ------------------------------------------------------------------
    def harvest_callback(self, request, response):
        if not request.start:
            response.success = False
            response.message = 'Harvesting not started'
            return response

        # 1. 调用检测服务
        detect_req = DetectBerries.Request()
        detect_req.start = True
        self.get_logger().info('Requesting berry detection...')
        detect_future = self.detect_client.call_async(detect_req)
        detect_future.add_done_callback(self.detection_done_callback)

        response.success = True
        response.message = 'Harvesting sequence started'
        return response

    def detection_done_callback(self, future):
        try:
            resp = future.result()
            berries = resp.berries.berries
            if not berries:
                self.get_logger().warn('No berries detected')
                return

            self.get_logger().info(f'Detected {len(berries)} ripe berries')
            for idx, berry in enumerate(berries):
                angles = self.calculate_joint_angles(berry.position)
                if angles is None:
                    self.get_logger().warn(f'Skipping berry {idx+1} - unreachable')
                    continue
                self.publish_target_angles(angles)
                self.get_logger().info(f'Harvesting berry {idx+1}')
            self.publish_target_angles(self.home_position)
            self.get_logger().info('已返回初始位置')
        except Exception as e:
            self.get_logger().error(f'Detection service call failed: {e}')

    # ------------------------------------------------------------------
    # 工具函数
    # ------------------------------------------------------------------
    def publish_target_angles(self, angles):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = angles
        msg.velocity = []
        msg.effort = []
        self.joint_pub.publish(msg)
        self.get_logger().info(f'Published target joint angles: {angles}')

    def calculate_joint_angles(self, berry_position):
        base_angle = math.atan2(berry_position.y, berry_position.x)
        planar_distance = math.sqrt(berry_position.x**2 + berry_position.y**2)
        target_x = planar_distance
        target_z = berry_position.z - self.base_height
        phi = math.pi/2  # 末端垂直向下
        angles = self.inverse_kinematics(
            x=target_x, y=target_z, phi=phi,
            L1=self.l1, L2=self.l2, L3=self.l3
        )
        self.get_logger().info(f'角度计算: {angles}')
        if angles is None:
            return None
        theta1, theta2, theta3 = angles
        joint_angles = [base_angle, theta1, theta2, theta3]

        # 限位检查
        names = ['base_rotation', 'joint1', 'joint2', 'joint3']
        for a, n in zip(joint_angles, names):
            lo, hi = self.joint_limits[n]
            if not (lo <= a <= hi):
                self.get_logger().warn(
                    f'{n} angle {a:.2f} out of range [{lo:.2f}, {hi:.2f}]'
                )
                return None
        
        return joint_angles

    # 逆运动学（与原代码相同）
    def inverse_kinematics(self, x, y, phi, L1, L2, L3):
        x3 = x - L3 * math.sin(phi)
        y3 = y - L3 * math.cos(phi)
        d = math.sqrt(x3**2 + y3**2)
        r_min = abs(L1 - L2)
        r_max = L1 + L2
        if not (r_min <= d <= r_max):
            self.get_logger().info(f'无法到达目标: {x3},{y3}')
            return None
        if x3 < 0 or y3 < 0:
            self.get_logger().warn('无法到达目标点2')
            return None
        cos_theta2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2 = math.acos(np.clip(cos_theta2, -1.0, 1.0))
        beta = math.atan2(y3, x3)
        cos_alpha = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
        alpha = math.acos(np.clip(cos_alpha, -1.0, 1.0))
        theta1 = math.pi/2 - (beta + alpha)
        theta3 = phi - (theta1 + theta2)
        if all(-math.pi/2 <= ang <= math.pi for ang in (theta1, theta2, theta3)):
            return theta1, theta2, theta3
        self.get_logger().warn('关节超限')
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