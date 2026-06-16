# 夹爪节点整体状态（一个节点管多个夹爪）
std_msgs/Header header
GripperState[] grippers   # 每个夹爪一项（通常 left + right）
bool estopped             # 是否处于软急停
