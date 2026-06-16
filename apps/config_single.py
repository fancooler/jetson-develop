"""单臂配置 — 继承公共基础配置，覆盖/新增单臂专用参数

用法：
  import config_single as config

适用脚本：arm.py / infer.py / runner.py
"""
from config import *        # noqa: F401,F403  导入全部公共参数
from config import _WORK_DIR  # 下划线开头不被 * 导出，显式引入

# ── 单臂选择 ──────────────────────────────────────────────────────────────────
ARM     = "A"   # 'A'=左臂, 'B'=右臂（传给 arm.py / TJArm）
ARM_IDX = 0     # A→0, B→1

# ── 准备位关节角（度）─────────────────────────────────────────────────────────
HOME_JOINTS = [89.0, -61.0, -88.0, -86.0, 61.0, -0.2, 2.6]   # 左臂准备位

# ── 单臂模型 ──────────────────────────────────────────────────────────────────
# TODO: 单臂模型尚未训练，路径为占位符；训练完成后更新此路径并将模型复制到 Jetson
MODEL_DIR      = os.path.join(_WORK_DIR, "groot_n15/models/tianjika_single")
EMBODIMENT_TAG = "isaaclab_franka"  # TODO: 确认单臂训练时使用的 embodiment tag

# ── 串口夹爪（单臂使用，具体型号和参数待确认）────────────────────────────────
# TODO: 确认夹爪型号和串口参数后更新以下字段
GRIPPER_MOCK      = True            # 暂时 Mock，待串口参数确认后改 False
GRIPPER_PORT      = "/dev/ttyUSB0"  # TODO: 确认实际串口设备
GRIPPER_BAUD      = 115200          # TODO: 确认波特率
GRIPPER_DEVICE_ID = 1               # TODO: 确认设备 ID
GRIPPER_VEL       = 50             # TODO: 确认速度（mm/s 或 %，视夹爪协议而定）
GRIPPER_TOR       = 50             # TODO: 确认扭矩/力（% 或 N）
GRIPPER_MAX_POS   = 85.0           # mm，对应完全张开

# ── 摄像头序列号（移到 config_camera.py，此处 re-export 保持向后兼容）────────
# 单臂不使用左腕摄像头
from config_camera import HEAD_CAM_SERIAL, WRIST_R_CAM_SERIAL  # noqa: F401, E402
