#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SimulationJointPublisher(Node):
    def __init__(self):
        super().__init__('simulation_joint_publisher')
        
        # 创建发布者，话题名必须为 /joint_states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # 订阅关节位置命令话题（使用JointState类型）
        self.command_sub = self.create_subscription(
            JointState, 
            '/joint_command',  # 命令话题名称
            self.command_callback, 
            10
        )
        
        # 定义关节名称（必须与URDF完全一致）
        self.joint_names = [
            'base_rotation_joint',
            'joint1',
            'joint2',
            'joint3'
        ]
        
        # 初始化关节位置 (默认值)
        self.joint_positions = [0.0, 0.0, 0.0, 2.0]
        
        # 设置定时器（50Hz）
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz
        self.get_logger().info("自定义关节状态发布节点已启动")
        self.get_logger().info(f"等待关节命令: /joint_command")
        self.get_logger().info(f"关节名称: {self.joint_names}")

    def command_callback(self, msg):
        """处理接收到的关节位置命令 - 仅使用位置数组"""
        # 检查位置数组长度是否匹配
        if len(msg.position) == len(self.joint_positions):
            # 直接复制整个位置数组
            self.joint_positions = list(msg.position)
            self.get_logger().debug(f"更新关节位置: {self.joint_positions}")
        else:
            self.get_logger().warn(
                f"位置数组长度不匹配! 期望 {len(self.joint_positions)} 个值, 收到 {len(msg.position)} 个值"
            )

    def publish_joint_states(self):
        """发布当前关节状态到 /joint_states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()  # 必须设置时间戳！
        msg.header.frame_id = "dummy_base"  # 可选：设置参考坐标系
        msg.name = self.joint_names
        msg.position = self.joint_positions
        
        # 发布消息
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimulationJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()