"""
infer_dual.py — GR00T N1.5 推理封装（双臂版，new_embodiment）

模型：tianji_optical_transfer_n15_3b_usdcam_lora64_long_4gpu_merged_checkpoint_20000

Action 输出格式（来自 DEPLOY.md）：
  action.right_arm[0:7]   : 右臂 7 个关节目标位置，弧度（绝对值，非增量）
  action.right_gripper[0] : 夹爪命令，>0.0=张开，≤0.0=闭合
  action.left_arm[0:7]    : 左臂 7 个关节目标位置，弧度（绝对值，非增量）
  action.left_gripper[0]  : 左爪命令，>0.0=张开，≤0.0=闭合

  extract_right/left_arm_cmd() 将弧度转换为度、再经 joint_map.urdf_to_sdk 换算到
  天机 SDK 关节约定后返回，runner 调用 da.move_joints('right'/'left', cmd.joints_deg)
  即可直接下发（无需再换算）。state.joint_pos 反向经 joint_map.sdk_to_urdf 还原为
  URDF 约定再喂模型。见 joint_map.py / memory: sim-real-coord-mismatch。

State 输入（TianjiDualArmDataConfig.state_keys）：
  提供：joint_pos[18]（弧度）
        right_ee_pose[7] · left_ee_pose[7]（base_link → 世界系变换后，[x,y,z,qw,qx,qy,qz]）
        right_gripper[1] · left_gripper[1]（URDF 关节位移，米）
        object_pose[7]（补固定值：训练数据均值位置）
  共 41 维

TODO（上机确认）：
  1. ✅ right_ee_pose/left_ee_pose[7] 格式：[x,y,z,qw,qx,qy,qz]（wxyz 约定）
  2. joint_pos[18] 布局：[右臂 0:7 | 左臂 7:14 | 右爪 14:16 | 左爪 16:18]（待上机确认）
  3. ✅ right_gripper action 极性：>0.0=张开，≤0.0=闭合
  4. object_pose 是否需要实时感知？目前补固定值（训练均值）
  5. ✅ EE pose 坐标系：Isaac Lab 世界系（已确认）
"""

import sys
import numpy as np
import cv2
import torch
from scipy.spatial.transform import Rotation

import config_dual as config
import joint_map   # URDF/模型 ↔ 天机 SDK 关节空间换算（见 memory: sim-real-coord-mismatch）

sys.path.insert(0, config.GROOT_SDK)

from gr00t.data.dataset import ModalityConfig
from gr00t.data.transform.base import ComposedModalityTransform
from gr00t.data.transform.concat import ConcatTransform
from gr00t.data.transform.state_action import StateActionToTensor, StateActionTransform
from gr00t.data.transform.video import VideoResize, VideoToNumpy, VideoToTensor
from gr00t.model.transforms import GR00TTransform
from gr00t.model.policy import Gr00tPolicy


# ── 常量 ──────────────────────────────────────────────────────────────────────

# 相机 key（与 modality.json video 字段一致）
VIDEO_KEYS = ["video.front", "video.left_wrist", "video.right_wrist"]

# State keys（与 TianjiDualArmDataConfig.state_keys 完全一致）
STATE_KEYS = [
    "state.joint_pos",      # [18]  关节角（弧度）
    "state.right_ee_pose",  # [7]   右臂 EE [x,y,z,qw,qx,qy,qz]，世界系
    "state.left_ee_pose",   # [7]   左臂 EE [x,y,z,qw,qx,qy,qz]，世界系
    "state.right_gripper",  # [1]   右爪 URDF 关节位移，米（~0.031-0.050m）
    "state.left_gripper",   # [1]   左爪 URDF 关节位移，米
    "state.object_pose",    # [7]   目标物体位姿，世界系 [x,y,z,qw,qx,qy,qz] — 补固定值
]

# Action keys（与 modality.json action 字段一致）
ACTION_KEYS = [
    "action.right_arm",      # [7] 右臂关节目标位置，弧度
    "action.right_gripper",  # [1]
    "action.left_arm",       # [7] 左臂关节目标位置，弧度
    "action.left_gripper",   # [1]
]

# 夹爪归一化参数（0=闭合/2mm，1=张开/85mm）
_GRIP_MIN_MM = 2.0
_GRIP_MAX_MM = 85.0

# right_gripper action 开关阈值（算法同事确认：-1=闭合，+1=张开）
GRIPPER_OPEN_THRESHOLD = 0.0   # >0.0 → 张开，≤0.0 → 闭合

# 机器人根坐标系 → 世界坐标系（Isaac Lab）变换
# 来源：DUAL_ARM_ROOT_ROT = (0.5, -0.5, 0.5, -0.5) wxyz，ROOT_POS = (0, 0, BASE_OFFSET)
# p_world = _R_ROOT_WORLD @ p_base + _ROOT_POS
# 等价展开：world_X = base_Z, world_Y = -base_X, world_Z = BASE_OFFSET - base_Y
_ROOT_POS     = np.array([0.0, 0.0, config.BASE_OFFSET], dtype=np.float64)
_R_ROOT_WORLD = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)

# 工具末端在 Link_7(右) / Link_16(左) 本地坐标系中的偏移（算法同事文档）
_RIGHT_TIP_LOCAL = np.array([0.245, 0.0, 0.004], dtype=np.float64)
_LEFT_TIP_LOCAL  = np.array([-0.245, 0.0, 0.004], dtype=np.float64)

# 物体位姿固定补偿值（世界系，[x,y,z,qw,qx,qy,qz]）
# 来自当前部署 checkpoint (tianji_optical_transfer_n15_3b_usdcam_lora64_long_4gpu_merged_checkpoint_20000)
# experiment_cfg/metadata.json statistics.state.object_pose.mean
# 注意：旧值 [0.32, -0.10, 0.84, 1, 0, 0, 0] 来自 debug 1-episode 模型，与当前 20000-step 模型不匹配。
# 若有实时物体检测，仍可在 get_action() 中传入 object_pose 参数覆盖此值。
_OBJECT_POSE_DUMMY = np.array(
    [0.10262330, -0.03748835, 0.89112371,
     0.22494638, -0.03030355, 0.24485634, -0.63757211], dtype=np.float32
)


# ── 模型加载 ──────────────────────────────────────────────────────────────────

def load_policy() -> Gr00tPolicy:
    """
    加载双臂 GR00T N1.5 推理策略（耗时，进程内只调一次）。

    配置来源：config.MODEL_DIR（指向合并后双臂模型目录）
              config.ACTION_HORIZON, config.DENOISING_STEPS
    """
    modality_config = {
        "video": ModalityConfig(
            delta_indices=[0],
            modality_keys=["front", "left_wrist", "right_wrist"],
        ),
        "state": ModalityConfig(
            delta_indices=[0],
            modality_keys=[
                "joint_pos",
                "right_ee_pose",
                "left_ee_pose",
                "right_gripper",
                "left_gripper",
                "object_pose",
            ],
        ),
        "action": ModalityConfig(
            delta_indices=list(range(config.ACTION_HORIZON)),
            modality_keys=["right_arm", "right_gripper", "left_arm", "left_gripper"],
        ),
        "annotation": ModalityConfig(
            delta_indices=[0],
            modality_keys=["human.action.task_description"],
        ),
    }

    modality_transform = ComposedModalityTransform(transforms=[
        VideoToTensor(apply_to=VIDEO_KEYS),
        VideoResize(apply_to=VIDEO_KEYS, height=224, width=224,
                    interpolation="linear"),
        VideoToNumpy(apply_to=VIDEO_KEYS),
        StateActionToTensor(apply_to=STATE_KEYS),
        StateActionTransform(
            apply_to=STATE_KEYS,
            normalization_modes={k: "min_max" for k in STATE_KEYS},
        ),
        StateActionToTensor(apply_to=ACTION_KEYS),
        StateActionTransform(
            apply_to=ACTION_KEYS,
            normalization_modes={k: "min_max" for k in ACTION_KEYS},
        ),
        ConcatTransform(
            video_concat_order=VIDEO_KEYS,
            state_concat_order=STATE_KEYS,
            action_concat_order=ACTION_KEYS,
        ),
        GR00TTransform(
            state_horizon=1,
            action_horizon=config.ACTION_HORIZON,
            max_state_dim=64,
            max_action_dim=32,
        ),
    ])

    policy = Gr00tPolicy(
        model_path=config.MODEL_DIR,
        embodiment_tag="new_embodiment",   # 双臂自定义 embodiment
        modality_config=modality_config,
        modality_transform=modality_transform,
        denoising_steps=config.DENOISING_STEPS,
        device="cuda",
    )
    policy.model.eval()
    return policy


# ── 辅助：坐标系转换 ──────────────────────────────────────────────────────────

def _rpy_to_quat_wxyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    RPY（弧度，XYZ 内旋）→ 四元数 [qw, qx, qy, qz]（Isaac Sim 约定）。

    TODO-1：若算法确认格式为 [x,y,z,w]，在 build_obs 内把 [w,x,y,z] 改为 roll(1,2,3,0)。
    """
    q = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()  # scipy: [x,y,z,w]
    qx, qy, qz, qw = q
    return np.array([qw, qx, qy, qz], dtype=np.float32)


def _gripper_mm_to_norm(pos_mm: float) -> float:
    """Xense 位置（mm）→ 归一化值 [0,1]，与训练时 right_gripper / left_gripper 一致。"""
    norm = (pos_mm - _GRIP_MIN_MM) / (_GRIP_MAX_MM - _GRIP_MIN_MM)
    return float(np.clip(norm, 0.0, 1.0))


def _joints_deg_to_rad(joints_deg: list | np.ndarray) -> np.ndarray:
    """机械臂 SDK 关节角（度）→ 弧度数组（7 维）。"""
    return np.deg2rad(np.asarray(joints_deg, dtype=np.float32))


def _build_joint_pos_18(
    joints_right_deg: list | np.ndarray,   # 右臂 7 关节，度
    joints_left_deg:  list | np.ndarray,   # 左臂 7 关节，度
    gripper_r_mm:     float,               # 右爪 Xense 位置，mm
    gripper_l_mm:     float,               # 左爪 Xense 位置，mm
) -> np.ndarray:
    """
    拼装 joint_pos[18]（弧度 / 米）。

    假设布局（TODO-2 待确认）：
      [0:7]   右臂 J1-J7，弧度
      [7:14]  左臂 J1-J7，弧度
      [14:16] 右爪两根手指（相对全开位置的偏移，[-0.05, 0]m）
      [16:18] 左爪两根手指（同上）

    夹爪填值依据：metadata 中 state.joint_pos[14:18] 范围 [-0.05, 0]，
    与 URDF 默认 prismatic [0, 0.05] 符号相反 → 训练时零点平移到 limit upper（全开）
    → 填值 = 单指位移(m) - 0.05（全开=0，全闭=-0.05）
    """
    from gripper_utils import xense_to_urdf
    # 机械臂读数是天机 SDK 关节约定 → 先换算回 URDF/模型约定，再转弧度喂 state。
    # （映射未标定/未启用时 sdk_to_urdf 恒等返回，等价改造前行为）
    r_arm = _joints_deg_to_rad(joint_map.sdk_to_urdf('right', joints_right_deg))  # (7,)
    l_arm = _joints_deg_to_rad(joint_map.sdk_to_urdf('left',  joints_left_deg))   # (7,)

    g_r_m = xense_to_urdf(gripper_r_mm) / 1000.0   # 单指位移，米 [0, 0.05]
    g_l_m = xense_to_urdf(gripper_l_mm) / 1000.0

    g_r_offset = g_r_m - 0.05                      # 相对全开偏移 [-0.05, 0]
    g_l_offset = g_l_m - 0.05

    return np.concatenate([
        r_arm,                                     # [0:7]
        l_arm,                                     # [7:14]
        [g_r_offset, g_r_offset],                  # [14:16] 两指对称
        [g_l_offset, g_l_offset],                  # [16:18] 两指对称
    ]).astype(np.float32)


def _build_ee_pose_7(
    pos_m:   np.ndarray | list,   # [x, y, z]，base_link 系，米
    rpy_rad: tuple | list,        # (roll, pitch, yaw)，弧度，base_link 系
) -> np.ndarray:
    """
    EE 位姿（base_link 系）→ [x, y, z, qw, qx, qy, qz]（世界系，shape [7]）。

    变换：p_world = _R_ROOT_WORLD @ p_base + _ROOT_POS
      _R_ROOT_WORLD = [[0,0,1],[-1,0,0],[0,-1,0]]  (from DUAL_ARM_ROOT_ROT wxyz=(0.5,-0.5,0.5,-0.5))
      _ROOT_POS     = [0, 0, BASE_OFFSET]
    等价展开：world_X = base_Z, world_Y = -base_X, world_Z = BASE_OFFSET - base_Y

    姿态也经同一旋转变换后转为四元数（Isaac Sim wxyz 约定）。
    """
    p_base = np.asarray(pos_m, dtype=np.float64)
    roll, pitch, yaw = rpy_rad

    # 位置：base_link → 世界系
    pos_world = _R_ROOT_WORLD @ p_base + _ROOT_POS

    # 姿态：base_link → 世界系
    R_ee_base  = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    R_ee_world = _R_ROOT_WORLD @ R_ee_base
    q = Rotation.from_matrix(R_ee_world).as_quat()   # scipy 格式: [x, y, z, w]
    qx, qy, qz, qw = q

    return np.concatenate([
        pos_world.astype(np.float32),
        np.array([qw, qx, qy, qz], dtype=np.float32),
    ])


# ── 观测构建 ──────────────────────────────────────────────────────────────────

def build_obs(
    front_rgb:         np.ndarray,         # 头部相机，任意分辨率，uint8
    left_wrist_rgb:    np.ndarray,         # 左腕相机，uint8
    right_wrist_rgb:   np.ndarray,         # 右腕相机，uint8
    joints_right_deg:  list | np.ndarray,  # 右臂 7 关节角，度
    joints_left_deg:   list | np.ndarray,  # 左臂 7 关节角，度
    fk_right:          tuple,              # fk_to_base() 返回 (pos_m, rpy_rad)
    fk_left:           tuple,              # fk_to_base() 返回 (pos_m, rpy_rad)
    gripper_r_mm:      float,              # 右爪 Xense 读数，mm
    gripper_l_mm:      float,              # 左爪 Xense 读数，mm
    task:              str | None = None,
    object_pose:       np.ndarray | None = None,  # 物体位姿 [x,y,z,qw,qx,qy,qz]，世界系
                                                   # None → 使用 _OBJECT_POSE_DUMMY
) -> dict:
    """
    硬件数据 → 模型输入 obs dict（与 TianjiDualArmDataConfig.state_keys 完全对齐）。

    Args:
        front_rgb:        头部（前向）相机 RGB
        left_wrist_rgb:   左腕相机 RGB
        right_wrist_rgb:  右腕相机 RGB
        joints_right_deg: 右臂 [J1..J7]，度（SDK read_joints()['right']）
        joints_left_deg:  左臂 [J1..J7]，度（SDK read_joints()['left']）
        fk_right:         frame_transform.fk_to_base(fk_output, 'right') 的返回值
                          = (pos_m [x,y,z], rpy_rad (roll,pitch,yaw))
        fk_left:          同上，左臂
        gripper_r_mm:     右爪 Xense 位置，mm → 内部转 URDF 米值（~0.031-0.050m）
        gripper_l_mm:     左爪 Xense 位置，mm → 内部转 URDF 米值
        task:             任务描述，None 时用 config.TASK
        object_pose:      目标物体位姿，7 维，None 时补训练均值固定值

    Returns:
        obs dict，可直接传给 policy.get_action()
    """
    from gripper_utils import xense_to_urdf

    # ── 图像（保持原始分辨率传入，VideoToTensor 验证 640×480，VideoResize 负责缩放到 224×224）

    # ── EE 位姿（base_link → 世界系）────────────────────────────────────────
    pos_r, rpy_r = fk_right
    pos_l, rpy_l = fk_left
    ee_right = _build_ee_pose_7(pos_r, rpy_r)  # (7,) 世界系
    ee_left  = _build_ee_pose_7(pos_l, rpy_l)  # (7,) 世界系

    # ── 关节位置（18 维，弧度 + URDF 夹爪位移米）────────────────────────────
    joint_pos_18 = _build_joint_pos_18(
        joints_right_deg, joints_left_deg, gripper_r_mm, gripper_l_mm
    )  # (18,) float32

    # ── 夹爪状态：URDF 关节位移，米（训练数据范围 ~0.031-0.050m）────────────
    # xense_to_urdf: Xense mm → URDF mm（单指行程 0-50mm），再 /1000 → 米
    grip_r_m = float(xense_to_urdf(gripper_r_mm)) / 1000.0
    grip_l_m = float(xense_to_urdf(gripper_l_mm)) / 1000.0

    # ── 物体位姿 ──────────────────────────────────────────────────────────────
    if object_pose is None:
        obj_pose = _OBJECT_POSE_DUMMY
    else:
        obj_pose = np.asarray(object_pose, dtype=np.float32)

    return {
        # 视频：(1, H, W, 3) uint8
        "video.front":       front_rgb[np.newaxis].astype(np.uint8),
        "video.left_wrist":  left_wrist_rgb[np.newaxis].astype(np.uint8),
        "video.right_wrist": right_wrist_rgb[np.newaxis].astype(np.uint8),

        # 状态：(T, dim) = (1, dim) float32，T=1 为当前帧
        "state.joint_pos":     joint_pos_18[np.newaxis],              # (1, 18)
        "state.right_ee_pose": ee_right[np.newaxis],                  # (1, 7)
        "state.left_ee_pose":  ee_left[np.newaxis],                   # (1, 7)
        "state.right_gripper": np.array([[grip_r_m]], dtype=np.float32),  # (1, 1)
        "state.left_gripper":  np.array([[grip_l_m]], dtype=np.float32),  # (1, 1)
        "state.object_pose":   obj_pose[np.newaxis],                  # (1, 7)

        # 任务描述
        "annotation.human.action.task_description": [task or config.TASK],
    }


# ── 推理接口 ──────────────────────────────────────────────────────────────────

def get_action(
    policy:           Gr00tPolicy,
    front_rgb:        np.ndarray,
    left_wrist_rgb:   np.ndarray,
    right_wrist_rgb:  np.ndarray,
    joints_right_deg: list | np.ndarray,
    joints_left_deg:  list | np.ndarray,
    fk_right:         tuple,
    fk_left:          tuple,
    gripper_r_mm:     float,
    gripper_l_mm:     float,
    task:             str | None = None,
) -> dict:
    """
    端到端推理。返回 action dict，key 见 ACTION_KEYS，
    每个值 shape = (ACTION_HORIZON, dim)。

    典型用法（在 runner 中）：
        actions = get_action(policy, ...)
        cmd = extract_right_arm_cmd(actions, step=0)
        # cmd.joints_deg  → da.move_joints('right', cmd.joints_deg.tolist())
        # cmd.gripper_open → True/False
    """
    obs = build_obs(
        front_rgb, left_wrist_rgb, right_wrist_rgb,
        joints_right_deg, joints_left_deg,
        fk_right, fk_left,
        gripper_r_mm, gripper_l_mm,
        task,
    )
    with torch.no_grad():
        return policy.get_action(obs)


# ── Action 解析 ───────────────────────────────────────────────────────────────

class ArmCmd:
    """单步单臂指令（从 action dict 中解析）。"""
    def __init__(self, joints_deg: np.ndarray, gripper_open: bool):
        self.joints_deg   = joints_deg     # [j1..j7]，度，已换算到 SDK 约定，直接下发
        self.gripper_open = gripper_open   # True=张开，False=闭合

# 向后兼容别名
RightArmCmd = ArmCmd


def extract_right_arm_cmd(actions: dict, step: int = 0) -> ArmCmd:
    """
    从 get_action() 返回的 action dict 中提取第 step 步的右臂指令。

    Args:
        actions:  get_action() 返回值
        step:     时间步索引，[0, ACTION_HORIZON)

    Returns:
        ArmCmd(joints_deg, gripper_open)
        joints_deg: [j1..j7]，度，可直接传给 da.move_joints('right', ...)
        gripper_open: True=张开，False=闭合
    """
    right_arm  = np.asarray(actions["action.right_arm"])      # (ACTION_HORIZON, 7)
    right_grip = np.asarray(actions["action.right_gripper"])  # (ACTION_HORIZON, 1)

    joints_rad      = right_arm[step, :7].astype(np.float64)
    joints_deg_urdf = np.rad2deg(joints_rad)                  # 模型输出是 URDF/模型约定
    # 换算到天机 SDK 约定再下发（映射未标定/未启用时恒等）
    joints_deg = joint_map.urdf_to_sdk('right', joints_deg_urdf).astype(np.float32)

    grip_val     = float(right_grip[step, 0])
    gripper_open = grip_val > GRIPPER_OPEN_THRESHOLD

    return ArmCmd(joints_deg, gripper_open)


def extract_left_arm_cmd(actions: dict, step: int = 0) -> ArmCmd:
    """
    从 get_action() 返回的 action dict 中提取第 step 步的左臂指令。

    Returns:
        ArmCmd(joints_deg, gripper_open)
        joints_deg: [j1..j7]，度，可直接传给 da.move_joints('left', ...)
    """
    left_arm  = np.asarray(actions["action.left_arm"])      # (ACTION_HORIZON, 7)
    left_grip = np.asarray(actions["action.left_gripper"])  # (ACTION_HORIZON, 1)

    joints_rad      = left_arm[step, :7].astype(np.float64)
    joints_deg_urdf = np.rad2deg(joints_rad)                 # 模型输出是 URDF/模型约定
    # 换算到天机 SDK 约定再下发（映射未标定/未启用时恒等）
    joints_deg = joint_map.urdf_to_sdk('left', joints_deg_urdf).astype(np.float32)

    grip_val     = float(left_grip[step, 0])
    gripper_open = grip_val > GRIPPER_OPEN_THRESHOLD

    return ArmCmd(joints_deg, gripper_open)


# ── 单独运行：验证模型加载与推理耗时 ─────────────────────────────────────────

if __name__ == "__main__":
    import time
    import logging
    logging.basicConfig(
        format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
        datefmt='%H:%M:%S', level=logging.INFO,
    )

    print("=== GR00T N1.5 双臂推理延迟测试 ===")
    print("加载模型 ...")
    t0 = time.time()
    policy = load_policy()
    print(f"模型加载完成  {(time.time()-t0)*1000:.0f}ms\n")

    # ── 构造随机 dummy 输入 ────────────────────────────────────────────────────
    rng = np.random.default_rng(42)

    front       = rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)
    left_wrist  = rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)
    right_wrist = rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)

    joints_r = [-89.0, -61.0, 88.0, -86.0, -61.0, -0.2, -2.6]  # 右臂准备位
    joints_l = [ 89.0, -61.0,-88.0, -86.0,  61.0, -0.2,  2.6]  # 左臂准备位

    # 用 frame_transform 生成合理的 FK 输出（dummy，假设 HOME FK）
    from frame_transform import fk_to_base
    fk_r_dummy = [430.0,  285.0, 325.0, 10.0, 5.0, -170.0]
    fk_l_dummy = [430.0, -285.0, 325.0,-10.0, 5.0,  170.0]
    fk_right = fk_to_base(fk_r_dummy, 'right')
    fk_left  = fk_to_base(fk_l_dummy, 'left')

    grip_r = 85.0   # mm，全开
    grip_l = 85.0

    # 热身
    _ = get_action(policy, front, left_wrist, right_wrist,
                   joints_r, joints_l, fk_right, fk_left, grip_r, grip_l)

    # 计时 5 次
    times = []
    for i in range(5):
        t0 = time.time()
        actions = get_action(policy, front, left_wrist, right_wrist,
                             joints_r, joints_l, fk_right, fk_left, grip_r, grip_l)
        times.append((time.time() - t0) * 1000)
        print(f"  第{i+1}次: {times[-1]:.1f}ms")

    print(f"\n平均推理延迟: {sum(times)/len(times):.1f}ms")

    print("\n=== Action 输出（step 0）===")
    for k in ACTION_KEYS:
        v = np.asarray(actions[k])
        print(f"  {k:<28} shape={v.shape}  step0={v[0]}")

    rcmd = extract_right_arm_cmd(actions, step=0)
    lcmd = extract_left_arm_cmd(actions, step=0)
    print(f"\n  右臂 joints_deg : {np.round(rcmd.joints_deg, 2)} °")
    print(f"  右爪 gripper_open: {rcmd.gripper_open}")
    print(f"  左臂 joints_deg : {np.round(lcmd.joints_deg, 2)} °")
    print(f"  左爪 gripper_open: {lcmd.gripper_open}")
