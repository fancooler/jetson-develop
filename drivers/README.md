# drivers — 传感器 / 执行器驱动封装

本目录存放各类硬件驱动的抽象封装层，供 ros2/ 和 inference/ 共同使用。

当前状态：占位，待重构。

规划：
- `arm/`      — 天机机械臂驱动封装（当前在 ros2/src/tj_marvin_driver/）
- `gripper/`  — Xense 夹爪驱动封装（当前在 ros2/src/xense_gripper_driver/）
- `camera/`   — RealSense 相机驱动封装
- `force/`    — 六维力传感器抽象层（WrenchSource，当前在 inference/app/）
- `tactile/`  — 视触觉传感器驱动（待开发）
