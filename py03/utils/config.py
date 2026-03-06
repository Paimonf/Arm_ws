# coding:utf-8

# 系统名称
SYSTEM_NAME = "用于智能采摘机器人的蓝莓成熟度识别系统"
# 开发者信息
DEVELOPER_NAME = "蓝莓组"
# 机构/店铺名称
ORG_NAME = "蓝莓"

# 图片及视频检测结果保存路径
save_path = 'save'

# 使用的模型路径
model_path = '/home/sh/Arm_ws/best.pt'

# 类别名称映射
names = {
    0: 'lm-H',
    1: 'lm-M',
    2: 'lm-L',
}

CH_names = [
    '蓝莓-高熟', '蓝莓-中熟', '蓝莓-低熟',
]
