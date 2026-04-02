import ikpy
from ikpy.chain import Chain
import numpy as np

# 从 URDF 加载模型
my_chain = Chain.from_urdf_file("/home/sh/Arm_ws/src/arm_description/urdf/blueberry_arm.urdf.xacro")
print(my_chain.links)
# 目标位置
target_position = [0.33, -0.05, 0.16]

# 计算 IK
joint_angles = my_chain.inverse_kinematics(target_position)
print(joint_angles)