# 流式夹爪指令（高频下发，非阻塞）。经 /gripper/command 发布（QoS BestEffort, KeepLast 1）。
string side           # "left" | "right" | "both"
float64 position      # 目标位置 mm（约 2=闭合 ~ 85=张开），节点内会 clip 到合法量程
float64 max_effort    # 最大力 N，<=0 用节点默认
