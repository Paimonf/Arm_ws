# ArmStatus.msg

# 标准消息头
std_msgs/Header header

# 连接状态
bool connected

# 就绪状态（已同步且无错误）
bool ready

# 是否正在执行轨迹
bool trajectory_active

# 错误代码
int32 error_code