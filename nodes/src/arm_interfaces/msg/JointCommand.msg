# 流式关节目标（高频连续下发，非点到点）。单位：度，与 MoveToJoints 一致。
# 经 /arm/joint_command 发布（QoS BestEffort, KeepLast 1）；需先用
# /arm/enable_streaming 开启流式模式，开启时离散运动动作会被拒（互斥）。
string arm                # "left" | "right" | "both"
float64[] joints_left     # 长度 7（arm 含 left/both 时必填）
float64[] joints_right    # 长度 7（arm 含 right/both 时必填）
