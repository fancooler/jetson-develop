"""公共基础配置 — 单臂和双臂共用的参数

具体模式请导入子配置：
  双臂：import config_dual as config
  单臂：import config_single as config

本文件仅供 frame_transform.py / fk_candidates.py / test_circle.py 等
不区分单双臂的工具脚本直接使用。
"""
import os
import math

# ── 路径（相对于 app/ 的上一级目录，ThinkBook/Jetson 均适用）────────────────
_WORK_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ── 机械臂连接 ────────────────────────────────────────────────────────────────
ROBOT_IP  = "192.168.1.190"
TJ_SDK    = os.path.join(_WORK_DIR, "TJ_marvin/TJ_FX_ROBOT_CONTRL_SDK-master")
CFG_FILE  = os.path.join(TJ_SDK, "ccs_m6_40.MvKDCfg")
VEL_RATIO      = 10    # 速度百分比（正式使用前先低速调试）
ACC_RATIO      = 10
REACH_TOL      = 0.5   # 到位判断阈值（度）
HOME_VEL_RATIO = 10    # go_home 速度（与推理时同速，谨慎到新 HOME 时降速）

# ── 控制模式 ──────────────────────────────────────────────────────────────────
# 'position'  : 纯位置跟随（SetJointMode），碰到障碍物会硬顶
# 'impedance' : 笛卡尔空间阻抗（SetImpCartMode），碰到阻力会顺应，更安全
CTRL_MODE = 'position'

# 阻抗参数（仅 CTRL_MODE='impedance' 时生效）
# K[0:3] 平移刚度 N/m，K[3:6] 旋转刚度 Nm/rad，K[6] 零空间刚度
# D[0:7] 阻尼比 0~1（0.8 = 略过阻尼，平稳不振荡）
IMP_K = [2000.0, 2000.0, 2000.0,   # Kx, Ky, Kz
          100.0,  100.0,  100.0,   # Krx, Kry, Krz
           20.0]                   # K_null
IMP_D = [0.8, 0.8, 0.8,            # 平移阻尼比
         0.8, 0.8, 0.8,            # 旋转阻尼比
         0.8]                      # 零空间阻尼比

# ── 摄像头公共参数（移到 config_camera.py，此处 re-export 保持向后兼容）────────
from config_camera import CAM_WIDTH, CAM_HEIGHT, CAM_FPS  # noqa: F401, E402

# ── GR00T 公共参数 ────────────────────────────────────────────────────────────
GROOT_SDK       = os.path.join(_WORK_DIR, "groot_n15/sdk")
TASK            = "Transfer the optical module from the right tray to the middle handoff pad, then to the left tray."
DENOISING_STEPS = 4
ACTION_HORIZON  = 16   # GR00T 每次推理输出的动作步数
EXEC_STEPS      = 1    # 每次推理后实际执行的步数（1=只执行第一步，16=执行全部）

# True = 运行真实 GR00T 推理但用 MockPolicy 动作控制机械臂（时序测试）
# False = 使用 GR00T 推理结果直接控制机械臂（正式运行）
# 注：双臂入口（runner_dual.py）使用 config_dual.MOCK_ACTIONS 单独覆盖此值
MOCK_ACTIONS = False
EXEC_HZ      = 20   # 动作执行频率（Hz），对应 50ms/步

# ── 坐标系转换 ────────────────────────────────────────────────────────────────
# 天机基座在 Isaac Lab 世界坐标系（= Franka 基座坐标系）中的 Z 高度
BASE_OFFSET = 0.8409087753   # 单位：米

# ── 单位换算 ──────────────────────────────────────────────────────────────────
# GR00T 训练数据：XYZ 单位为米，姿态单位为弧度
# 天机 SDK IK 输入：XYZ 单位为毫米，姿态（ABC）单位为度
#
# FK 输出 XYZABC 中，ABC 为 ZYX 欧拉角（A=绕Z，B=绕Y，C=绕X），
# 对应 GR00T 的 [yaw, pitch, roll]。
ABC_TO_RPY = [2, 1, 0]   # FK[A,B,C] → GR00T[roll,pitch,yaw] 的索引映射
                          # 当前: roll←C, pitch←B, yaw←A

def mm_deg_to_m_rad(xyzabc_mm_deg):
    """FK 输出 [X_mm, Y_mm, Z_mm, A_deg, B_deg, C_deg] → GR00T 输入格式"""
    x, y, z, a, b, c = xyzabc_mm_deg
    abc = [a, b, c]
    roll  = math.radians(abc[ABC_TO_RPY[0]])
    pitch = math.radians(abc[ABC_TO_RPY[1]])
    yaw   = math.radians(abc[ABC_TO_RPY[2]])
    return [x / 1000.0, y / 1000.0, z / 1000.0, roll, pitch, yaw]

def m_rad_to_mm_deg(xyzrpy_m_rad):
    """GR00T 输出 [x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad] → IK 输入格式"""
    x, y, z, roll, pitch, yaw = xyzrpy_m_rad
    rpy = [roll, pitch, yaw]
    abc = [0.0, 0.0, 0.0]
    abc[ABC_TO_RPY[0]] = math.degrees(rpy[0])
    abc[ABC_TO_RPY[1]] = math.degrees(rpy[1])
    abc[ABC_TO_RPY[2]] = math.degrees(rpy[2])
    return [x * 1000.0, y * 1000.0, z * 1000.0] + abc
