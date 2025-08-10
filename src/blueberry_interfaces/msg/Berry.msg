# 单个蓝莓的检测结果
int32 id                   # 唯一标识符
geometry_msgs/Point position  # 3D位置 (米)
float32 confidence         # 检测置信度 [0-1]
bool is_ripe               # 是否成熟
float32 diameter           # 估算直径(米)