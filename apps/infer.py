"""
infer.py — GR00T N1.5 推理封装

职责：
  模型加载 + 推理接口，屏蔽 gr00t SDK 细节，不含任何硬件控制逻辑。

对外接口：
  policy  = load_policy()
  actions = get_action(policy, head_rgb, wrist_rgb, ee_state, gripper_pos_mm)

obs/action key 约定（与训练 metadata 一致）：
  VIDEO_KEYS  : ["video.image", "video.wrist_image"]
  STATE_KEYS  : ["state.x","state.y","state.z","state.roll","state.pitch","state.yaw","state.gripper"]
  ACTION_KEYS : ["action.x","action.y","action.z","action.roll","action.pitch","action.yaw","action.gripper"]

坐标系说明（build_obs 内部处理）：
  GR00T 训练使用 Isaac Sim / Franka 世界坐标系，
  天机基座坐标系与其存在固定偏移（config.BASE_OFFSET）和 yaw 旋转（-π）。
  这里做简化线性补偿；精确变换见 frame_transform.py。

TODO: 等算法同事确认 metadata.json key 名称后，核对 STATE_KEYS / ACTION_KEYS 与
      训练数据完全一致（尤其是 gripper 维度和 state/action 的顺序）。
"""

import sys
import math
import numpy as np
import cv2
import torch

import config_single as config

sys.path.insert(0, config.GROOT_SDK)

from gr00t.data.dataset import ModalityConfig
from gr00t.data.transform.base import ComposedModalityTransform
from gr00t.data.transform.concat import ConcatTransform
from gr00t.data.transform.state_action import StateActionToTensor, StateActionTransform
from gr00t.data.transform.video import VideoResize, VideoToNumpy, VideoToTensor
from gr00t.model.transforms import GR00TTransform
from gr00t.model.policy import Gr00tPolicy


# ── Key 约定（模块级常量，供 runner.py 导入）──────────────────────────────────

VIDEO_KEYS = ["video.image", "video.wrist_image"]
STATE_KEYS = [
    "state.x", "state.y", "state.z",
    "state.roll", "state.pitch", "state.yaw",
    "state.gripper",
]
ACTION_KEYS = [
    "action.x", "action.y", "action.z",
    "action.roll", "action.pitch", "action.yaw",
    "action.gripper",
]


# ── 模型加载 ──────────────────────────────────────────────────────────────────

def load_policy() -> Gr00tPolicy:
    """
    加载并返回 GR00T N1.5 推理策略（耗时，进程内只调一次）。

    配置来源：config.MODEL_DIR, config.EMBODIMENT_TAG,
              config.ACTION_HORIZON, config.DENOISING_STEPS
    """
    modality_config = {
        "video": ModalityConfig(
            delta_indices=[0],
            modality_keys=["image", "wrist_image"],
        ),
        "state": ModalityConfig(
            delta_indices=[0],
            modality_keys=["x", "y", "z", "roll", "pitch", "yaw", "gripper"],
        ),
        "action": ModalityConfig(
            delta_indices=list(range(config.ACTION_HORIZON)),
            modality_keys=["x", "y", "z", "roll", "pitch", "yaw", "gripper"],
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
        StateActionTransform(apply_to=STATE_KEYS,
                             normalization_modes={k: "min_max" for k in STATE_KEYS}),
        StateActionToTensor(apply_to=ACTION_KEYS),
        StateActionTransform(apply_to=ACTION_KEYS,
                             normalization_modes={k: "min_max" for k in ACTION_KEYS}),
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
        embodiment_tag=config.EMBODIMENT_TAG,
        modality_config=modality_config,
        modality_transform=modality_transform,
        denoising_steps=config.DENOISING_STEPS,
        device="cuda",
    )
    policy.model.eval()
    return policy


# ── 观测构建 ──────────────────────────────────────────────────────────────────

def build_obs(head_rgb:      np.ndarray,
              wrist_rgb:     np.ndarray,
              ee_state:      list | np.ndarray,
              gripper_pos_mm: float,
              task:          str | None = None,
              ) -> dict:
    """
    传感器数据 → 模型输入 obs dict。

    Args:
        head_rgb:       头部相机 RGB 图像，任意分辨率，uint8
        wrist_rgb:      腕部相机 RGB 图像，任意分辨率，uint8
        ee_state:       末端位姿 [x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad]，
                        天机基座坐标系
        gripper_pos_mm: 当前夹爪位置，Xense TCP 读数，mm（0=闭合，85=全开）
        task:           任务描述字符串，None 时使用 config.TASK

    Returns:
        obs dict，可直接传给 policy.get_action()
    """
    # 图像 resize 到模型输入尺寸（256×256）
    head_rgb  = cv2.resize(head_rgb,  (256, 256), interpolation=cv2.INTER_LINEAR)
    wrist_rgb = cv2.resize(wrist_rgb, (256, 256), interpolation=cv2.INTER_LINEAR)

    x, y, z, roll, pitch, yaw = ee_state

    # 坐标系补偿：天机基座系 → GR00T 训练使用的 Isaac Sim / Franka 世界系
    #   z_f   = z_tianji + BASE_OFFSET
    #   yaw_f = yaw_tianji - π
    # 注：模型输出为增量动作，增量叠加时偏移自动抵消。
    #     精确的旋转矩阵变换见 frame_transform.py（上机验证后切换）。
    z_f   = z   + config.BASE_OFFSET
    yaw_f = yaw - math.pi

    # 夹爪状态：Xense mm → URDF 对称关节对 [g_left, g_right]（单位：m）
    # URDF 单指行程 0~50mm；两关节对称，正向 +Y，负向 -Y，量级约 ±0.032m
    from gripper_utils import xense_to_urdf
    g_joint_mm = xense_to_urdf(gripper_pos_mm)   # 单指位移，mm
    g_m = g_joint_mm / 1000.0                    # 转米
    g_l, g_r = g_m, -g_m                         # 对称关节

    return {
        "video.image":       head_rgb[np.newaxis].astype(np.uint8),
        "video.wrist_image": wrist_rgb[np.newaxis].astype(np.uint8),
        "state.x":           np.array([[x]],        dtype=np.float32),
        "state.y":           np.array([[y]],        dtype=np.float32),
        "state.z":           np.array([[z_f]],      dtype=np.float32),
        "state.roll":        np.array([[roll]],     dtype=np.float32),
        "state.pitch":       np.array([[pitch]],    dtype=np.float32),
        "state.yaw":         np.array([[yaw_f]],    dtype=np.float32),
        "state.gripper":     np.array([[g_l, g_r]], dtype=np.float32),
        "annotation.human.action.task_description": [task or config.TASK],
    }


# ── 推理接口 ──────────────────────────────────────────────────────────────────

def get_action(policy:         Gr00tPolicy,
               head_rgb:       np.ndarray,
               wrist_rgb:      np.ndarray,
               ee_state:       list | np.ndarray,
               gripper_pos_mm: float,
               task:           str | None = None,
               ) -> dict:
    """
    端到端推理：传感器数据 → 动作 dict（含 ACTION_HORIZON 步）。

    Args:
        policy:         load_policy() 返回的策略对象
        head_rgb:       头部相机 RGB，uint8
        wrist_rgb:      腕部相机 RGB，uint8
        ee_state:       [x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad]
        gripper_pos_mm: 夹爪 Xense 位置，mm
        task:           任务描述，None 时用 config.TASK

    Returns:
        dict，key 为 ACTION_KEYS，值 shape = (ACTION_HORIZON,) 或 (ACTION_HORIZON, dim)
    """
    obs = build_obs(head_rgb, wrist_rgb, ee_state, gripper_pos_mm, task)
    with torch.no_grad():
        return policy.get_action(obs)


# ── 单独运行：验证模型加载与推理耗时 ─────────────────────────────────────────

if __name__ == "__main__":
    import time
    import logging
    logging.basicConfig(
        format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
        datefmt='%H:%M:%S', level=logging.INFO,
    )

    print("=== GR00T N1.5 推理延迟测试 ===")

    print("加载模型 ...")
    t0 = time.time()
    policy = load_policy()
    print(f"模型加载完成  {(time.time()-t0)*1000:.0f}ms\n")

    # 构造随机输入（模拟 RealSense 图像 + 臂状态）
    head  = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    wrist = np.random.randint(0, 255, (240, 424, 3), dtype=np.uint8)
    ee    = [0.493, -0.006, 0.110, 0.567, 0.053, -2.159]
    grip  = 42.5   # mm

    # 热身
    _ = get_action(policy, head, wrist, ee, grip)

    # 计时 5 次
    times = []
    for i in range(5):
        t0 = time.time()
        actions = get_action(policy, head, wrist, ee, grip)
        times.append((time.time() - t0) * 1000)
        print(f"  第{i+1}次: {times[-1]:.1f}ms")

    print(f"\n平均推理延迟: {sum(times)/len(times):.1f}ms")

    print("\n=== 输出 shape ===")
    for k in ACTION_KEYS:
        v = np.array(actions[k])
        print(f"  {k}: shape={v.shape}  step0={v.flat[0]:.4f}")
