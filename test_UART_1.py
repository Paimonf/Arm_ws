import ikpy
from ikpy.chain import Chain
import numpy as np

# 加载 URDF
chain = Chain.from_urdf_file("/home/sh/Arm_ws/src/arm_description/urdf/blueberry_arm.urdf.xacro")

# ikpy 解
ikpy_solution = [0,-0.150371428, 1.40085494, 0, 0, 0]  # 注意长度6，含固定关节
fk_pos_ikpy = chain.forward_kinematics(ikpy_solution)[:3, 3]
print("ikpy 解对应的末端位置:", fk_pos_ikpy)

# 解析法解（需扩展为6关节，固定关节置0）
analytic_solution = [0,-0.1504, 0.7616, 1.0956, -0.2873, 0]  # 按顺序 base, j1, j2, j3, 固定, 固定
fk_pos_analytic = chain.forward_kinematics(analytic_solution)[:3, 3]
print("解析法解对应的末端位置:", fk_pos_analytic)

print("目标位置:", [0.33, -0.05, 0.16])