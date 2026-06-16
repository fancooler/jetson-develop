"""双臂配置 — 继承公共基础配置，覆盖/新增双臂专用参数

用法：
  import config_dual as config

适用脚本：arm_utils.py / infer_dual.py / runner_dual.py / demo.py / demo_right.py
"""
from config import *        # noqa: F401,F403  导入全部公共参数
from config import _WORK_DIR  # 下划线开头不被 * 导出，显式引入

# ── 双臂模型 ──────────────────────────────────────────────────────────────────
MODEL_DIR      = os.path.join(_WORK_DIR, "groot_n15/models/tianjika_n15")
# ↑ 双臂合并模型（tianji_optical_transfer，safetensors 格式）
EMBODIMENT_TAG = "isaaclab_franka"

# ── 运行模式 ──────────────────────────────────────────────────────────────────
# 覆盖 config.MOCK_ACTIONS（双臂入口专用，与单臂入口独立控制）
# True=真实推理但不下发机械臂指令（流程调试）；False=正式运行
MOCK_ACTIONS = False

# ── 关节空间映射（URDF/模型 ↔ 天机 SDK）─────────────────────────────────────────
# 见 joint_map.py / memory: sim-real-coord-mismatch。
#   True  → action(URDF)→SDK 下发、SDK→URDF 喂 state、HOME(URDF)→SDK 都经 joint_map 换算
#   False → 全程恒等（改造前行为，A/B 对照用）
# 注意：即便为 True，若 test/joint_map.json 尚未由 calib_joints.py 生成，joint_map
#       也会退回恒等并打 WARNING。推荐上机顺序：
#         1) python3 test/calib_joints.py --save  （离线，残差应≈0）
#         2) python3 test/replay_arm.py --arm right --map  （单臂慢速验证映射）
#         3) 确认无误后再跑 runner_dual.py（USE_JOINT_MAP=True 全管线生效）
USE_JOINT_MAP = False

# ── 准备位关节角（度）─────────────────────────────────────────────────────────
#
# 设计原则（已通过 fk_candidates.py 验证 + 上机确认）：
#   R_LEFT ≠ R_RIGHT（GRV 参数验证）：左臂 arm_Z→base_link -Z，右臂→+Z
#   对称规则：HOME_JOINTS_RIGHT 奇数轴（J1/J3/J5/J7）取反
#   结果：两臂末端 base_link 中 X,Y 对齐，Z 相差 74mm（两臂底座高度差）
#
# 左臂双 home 位：
#   home0 — 与右臂对称，两臂末端近似对齐，适合抓取任务开始前
#   home1 — 左臂收至机器人左后侧（J2/J4 更折叠），远离右臂工作空间，
#            用于右臂单独操作时停放左臂
# 2026-05-29：HOME 改为 metadata action.{right,left}_arm.mean（弧度→度）。
# 原因：旧 HOME J2=-61° 远离训练分布，state 输入超分布导致模型推理失常。
# 新 HOME 直接是训练 action 空间的均值，落在训练分布的核心区域。
#
# 数值来源：metadata.json statistics.action.right_arm.mean / left_arm.mean (rad)
#   R rad = [0.731, 0.065, 0.262, -1.386, 0.545, 0.252, 0.171]
#   L rad = [-0.912, -1.086, -1.727, -1.569, 0.307, -0.141, -0.614]
#
# 旧 HOME（回退时取消注释，替换下面两行）：
#   HOME_JOINTS_LEFT  = [ 89.0, -61.0, -88.0, -86.0,  61.0, -0.2,  2.6]
#   HOME_JOINTS_RIGHT = [-89.0, -61.0,  88.0, -86.0, -61.0, -0.2, -2.6]
HOME_JOINTS_LEFT  = [ 90.0, -75.0, -90.0, -95.0,  75.0,  0.0,  0.0]  # 左臂准备位
HOME_JOINTS_RIGHT = [-90.0, -75.0,  90.0, -95.0, -75.0,  0.0,  0.0]  # 右臂准备位

# ── Xense 夹爪（DDS/网络型）──────────────────────────────────────────────────
GRIPPER_MOCK       = False            # True=两侧均用 MockGripper（夹爪硬件故障，先 mock）
GRIPPER_MOCK_RIGHT = False            # True=右夹爪用 MockGripper（硬件故障时跳过）
# 夹爪 MAC：下面是默认/兜底值。若 robot_env.sh 已导出 $ROBOTS_YAML + $ROBOT_ID
# （.bashrc 里 source 过），则按 ROBOT_ID 从车队注册表 robots.yaml 的 grippers 段覆盖，
# 与相机同源、单一来源。下游 test_gripper.py / app/gripper.py / ROS2 夹爪节点自动生效。
GRIPPER_MAC_LEFT  = "3ad820773a85"  # 左臂夹爪 MAC（无冒号小写）默认/兜底
GRIPPER_MAC_RIGHT = "72a7da225db7"  # 右臂夹爪 MAC（无冒号小写）默认/兜底


def _load_gripper_macs_from_registry():
    """按 ROBOT_ID 从 $ROBOTS_YAML 的 grippers 段覆盖上面的默认 MAC。
    注册表缺失/无该机/无 grippers/无 yaml → 静默保留默认（不破坏现状）。返回来源 robotN 或 None。"""
    import os
    path = os.environ.get("ROBOTS_YAML")
    rid  = os.environ.get("ROBOT_ID")
    if not path or not rid:
        return None
    try:
        import yaml
        with open(path) as f:
            g = (yaml.safe_load(f) or {})["robots"][rid].get("grippers") or {}
    except Exception:
        return None
    global GRIPPER_MAC_LEFT, GRIPPER_MAC_RIGHT
    if g.get("left_mac"):
        GRIPPER_MAC_LEFT = str(g["left_mac"])
    if g.get("right_mac"):
        GRIPPER_MAC_RIGHT = str(g["right_mac"])
    return rid


_GRIPPER_MAC_SOURCE = _load_gripper_macs_from_registry()  # None=用默认；robotN=来自注册表
GRIPPER_VMAX      = 80.0            # 默认最大速度，mm/s（最小 40）
GRIPPER_FMAX      = 27.0            # 默认最大力，N
GRIPPER_TOL       = 2.0             # 到位容差，mm
GRIPPER_POS_OPEN  = 85.0            # 完全张开位置，mm
GRIPPER_POS_CLOSE = 2.0             # 完全闭合位置，mm（非 0，避免顶死触发过流保护）

# ── 工作空间禁区（base_link 坐标系，单位：米）────────────────────────────────
# 每项格式：(x_min, x_max, y_min, y_max, z_min, z_max)
# EE 目标落入任一禁区时，move_to_ee_base 拦截该指令
WORKSPACE_FORBIDDEN = [
    (0.40, 0.60, -0.10, 0.10, 0.00, 1.00),   # 柱子（实测后可微调边界）
]

# ── 摄像头序列号（移到 config_camera.py，此处 re-export 保持向后兼容）────────
from config_camera import (  # noqa: F401, E402
    HEAD_CAM_SERIAL, WRIST_L_CAM_SERIAL, WRIST_R_CAM_SERIAL,
)
