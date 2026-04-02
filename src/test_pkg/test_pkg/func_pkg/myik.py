"""
机械臂逆运动学求解模块
提供平面三连杆机械臂的几何逆解，以及从三维目标点计算完整关节角度的功能。
"""

import math

def inverse_kinematics_planar(x, z, phi, L1, L2, L3):
    """
    平面三连杆机械臂逆运动学（几何法）

    参数:
        x, z: 目标点坐标（机械臂平面内）
        phi: 末端执行器相对于水平方向的夹角（弧度），0表示水平向右，pi/2表示垂直向上
        L1, L2, L3: 连杆长度

    返回:
        (theta1, theta2, theta3) 弧度，若不可达则返回 None
    """
    try:
        # 计算腕部位置（第三个关节的末端，即末端执行器之前的点）
        wrist_x = x - L3 * math.sin(phi)
        wrist_z = z - L3 * math.cos(phi)

        # 腕部到原点的距离
        d = math.sqrt(wrist_x**2 + wrist_z**2)

        # 可达性检查
        if d > (L1 + L2) or d < abs(L1 - L2):
            return None

        # 计算 theta2（肘部角度，使用余弦定理）
        cos_theta2 = (wrist_x**2 + wrist_z**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_theta2 = max(min(cos_theta2, 1.0), -1.0)  # 防止数值误差
        theta2 = math.acos(cos_theta2)

        # 计算 theta1（肩部角度）
        alpha = math.atan2(wrist_z, wrist_x)
        beta = math.acos((L1**2 + d**2 - L2**2) / (2 * L1 * d))
        theta1 = math.pi / 2 - alpha - beta   # 注意坐标系定义

        # 计算 theta3
        theta3 = phi - theta1 - theta2

        return theta1, theta2, theta3
    except Exception:
        return None


def calculate_joint_angles(berry_x, berry_y, berry_z,
                           base_height, L1, L2, L3,
                           preferred_orientation=1.57,
                           attempts=10):
    """
    计算采摘蓝莓所需的完整关节角度（四自由度机械臂：基座旋转 + 三个俯仰关节）

    参数:
        berry_x, berry_y, berry_z: 目标点坐标（相对于 base_link）
        base_height: 基座高度（关节1相对于 base_link 的高度）
        L1, L2, L3: 连杆长度
        preferred_orientation: 首选末端方向（弧度），1.57 表示垂直向上
        attempts: 尝试不同方向的次数

    返回:
        [base_angle, theta1, theta2, theta3] 或 None
    """
    # 1. 基座旋转角度
    base_angle = math.atan2(berry_y, berry_x)

    # 2. 转换到机械臂平面（X-Z 平面）
    planar_distance = math.sqrt(berry_x**2 + berry_y**2)
    target_x = planar_distance
    target_z = berry_z - base_height   # z 方向向上为正

    best_solution = None
    best_orientation_diff = float('inf')

    # 尝试多种末端方向
    for attempt in range(attempts):
        # 生成不同的 phi 值：从首选方向开始，向两侧试探
        angle_variation = (attempt // 2) * 0.2 * (-1 if attempt % 2 == 0 else 1)
        phi = preferred_orientation + angle_variation

        # 调用平面逆运动学
        result = inverse_kinematics_planar(target_x, target_z, phi, L1, L2, L3)
        if result is None:
            continue

        theta1, theta2, theta3 = result
        joint_angles = [base_angle, theta1, theta2, theta3]

        # 评估优劣：使用与首选方向的差异
        orientation_diff = abs(phi - preferred_orientation)
        if orientation_diff < best_orientation_diff:
            best_orientation_diff = orientation_diff
            best_solution = joint_angles

    return best_solution


if __name__ == "__main__":
    # 示例：使用给定的机械臂参数测试目标点 [0.33, -0.05, 0.16]
    L1 = 0.129      # 关节1到关节2的长度
    L2 = 0.129      # 关节2到关节3的长度
    L3 = 0.121      # 关节3到末端执行器的长度
    base_height = 0.103  # 基座到关节1的高度

    target = [0.33, -0.05, 0.16]

    joint_angles = calculate_joint_angles(
        berry_x=target[0],
        berry_y=target[1],
        berry_z=target[2],
        base_height=base_height,
        L1=L1, L2=L2, L3=L3,
        preferred_orientation=1.57,  # 垂直向上
        attempts=10
    )

    if joint_angles:
        print("计算出的关节角度 (弧度):")
        names = ['base_rotation', 'joint1', 'joint2', 'joint3']
        for name, val in zip(names, joint_angles):
            print(f"  {name}: {val:.4f} ({math.degrees(val):.2f}°)")
    else:
        print("无解")