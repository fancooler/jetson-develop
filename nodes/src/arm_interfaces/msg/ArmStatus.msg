# 机械臂驱动状态（通用）
std_msgs/Header header
bool connected          # 是否已连接控制器
bool estopped           # 是否处于软急停
bool busy               # 是否有动作正在执行
bool streaming          # 是否处于流式控制模式（/arm/joint_command 生效中）
string ctrl_mode        # "position" | "impedance"
