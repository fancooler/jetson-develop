#!/usr/bin/env python3
"""
runner.py — GR00T N1.5 机器人控制主循环

职责：硬件初始化 → 传感器读取 → 调用 infer.get_action() → 执行动作 → 计时统计

config.MOCK_ACTIONS = True  → 真实推理 + MockPolicy 动作（时序测试，不动机械臂）
config.MOCK_ACTIONS = False → 真实推理 + GR00T 动作（正式运行）

模型推理逻辑见 infer.py。
"""

import math
import time
import logging

import numpy as np

logging.basicConfig(
    format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
    datefmt='%H:%M:%S',
    level=logging.INFO,
)
logger = logging.getLogger("runner")

import config_single as config
from infer   import load_policy, get_action, ACTION_KEYS
from arm     import TJArm
from gripper import Gripper, MockGripper
from camera  import DualCamera


# ── MockPolicy ────────────────────────────────────────────────────────────────

class MockPolicy:
    """在当前位置附近做小幅正弦振荡，用于时序测试（不替代真实推理，仅替换动作）。"""

    def __init__(self):
        self._t = 0.0

    def get_action(self) -> dict:
        dt  = 1.0 / config.EXEC_HZ
        amp = 0.005   # 5mm 振幅

        acts = {k: [] for k in ACTION_KEYS}
        for i in range(config.ACTION_HORIZON):
            t = self._t + i * dt
            acts["action.x"].append(amp * math.sin(2 * math.pi * 0.5 * t))
            acts["action.y"].append(0.0)
            acts["action.z"].append(amp * math.sin(2 * math.pi * 0.3 * t))
            acts["action.roll"].append(0.0)
            acts["action.pitch"].append(0.0)
            acts["action.yaw"].append(0.0)
            acts["action.gripper"].append(0.5)
        self._t += config.ACTION_HORIZON * dt
        return {k: np.array(v) for k, v in acts.items()}


# ── 动作解析 ──────────────────────────────────────────────────────────────────

def parse_step(actions: dict, step: int, ee_state: list) -> tuple:
    """
    从 action dict 取第 step 步，增量叠加到当前 ee_state，返回目标位姿 + 夹爪归一化值。

    BASE_OFFSET / yaw 旋转在增量叠加中自动抵消，无需显式处理。

    Args:
        actions:  get_action() 返回的 dict
        step:     要执行的步索引 (0 ~ ACTION_HORIZON-1)
        ee_state: 当前末端位姿 [x,y,z,roll,pitch,yaw]，天机基座系

    Returns:
        (x, y, z, roll, pitch, yaw, gripper_norm)
        gripper_norm: 0.0~1.0，供 gripper.set() 换算
    """
    def _get(key):
        v = np.atleast_1d(np.array(actions[key]))
        return float(v[step] if v.ndim == 1 else v[step, 0])

    cx, cy, cz, cr, cp, cyw = ee_state
    ag = np.array(actions["action.gripper"])
    g_norm = float(ag[step] if ag.ndim == 1 else ag[step].mean())

    return (
        cx  + _get("action.x"),
        cy  + _get("action.y"),
        cz  + _get("action.z"),
        cr  + _get("action.roll"),
        cp  + _get("action.pitch"),
        cyw + _get("action.yaw"),
        g_norm,
    )


# ── 主循环 ────────────────────────────────────────────────────────────────────

def _ts(label, t0):
    ms = (time.time() - t0) * 1000
    logger.info(f"  [初始化] {label}: {ms:.0f}ms")
    return ms


def main():
    mode = "MOCK_ACTIONS（真实推理+Mock动作）" if config.MOCK_ACTIONS else "正式运行"
    logger.info("=" * 60)
    logger.info(f"  GR00T N1.5 控制程序  模式={mode}")
    logger.info("=" * 60)

    # ── 初始化（计时）────────────────────────────────────────────────────────
    logger.info("[初始化开始]")

    t0 = time.time()
    arm = TJArm()
    if not arm.connect():
        logger.error("机械臂连接失败，退出")
        return
    _ts("机械臂连接", t0)

    t0 = time.time()
    gripper = MockGripper() if config.GRIPPER_MOCK else Gripper(
        port=config.GRIPPER_PORT,
        baud=config.GRIPPER_BAUD,
        device_id=config.GRIPPER_DEVICE_ID,
        default_vel=config.GRIPPER_VEL,
        default_tor=config.GRIPPER_TOR,
    )
    _ts("夹爪初始化", t0)

    t0 = time.time()
    cams = DualCamera()
    cams.start()
    _ts("摄像头初始化", t0)

    t0 = time.time()
    logger.info("  [初始化] 加载 GR00T N1.5 模型 ...")
    policy = load_policy()          # ← 来自 infer.py
    _ts("GR00T 模型加载", t0)

    mock_policy = MockPolicy() if config.MOCK_ACTIONS else None

    t0 = time.time()
    arm.go_home()
    _ts("回准备位", t0)

    logger.info("[初始化完成]")
    logger.info("=" * 60)
    logger.info(f"  控制循环开始  horizon={config.ACTION_HORIZON}  exec_hz={config.EXEC_HZ}")
    logger.info(f"  {'(推理结果将被 MockPolicy 替换)' if config.MOCK_ACTIONS else '(使用 GR00T 推理结果)'}")
    logger.info("=" * 60)

    step_dt       = 1.0 / config.EXEC_HZ
    cycle         = 0
    t_obs_list    = []
    t_infer_list  = []
    t_exec_list   = []
    t_cycle_list  = []

    try:
        while True:
            cycle += 1
            t_cycle_start = time.time()

            # 1. 读观测
            t0 = time.time()
            head_rgb, wrist_rgb = cams.read()
            ee_state            = arm.get_ee_state()
            gripper_pos_mm      = gripper.query()[0]
            t_obs = (time.time() - t0) * 1000

            # 2. GR00T 推理（始终执行，用于计时）
            t0 = time.time()
            groot_actions = get_action(         # ← 来自 infer.py
                policy, head_rgb, wrist_rgb, ee_state, gripper_pos_mm
            )
            t_infer = (time.time() - t0) * 1000

            # 3. 选择动作来源
            actions = mock_policy.get_action() if config.MOCK_ACTIONS else groot_actions

            # 4. 执行前 EXEC_STEPS 步
            t0 = time.time()
            for step in range(config.EXEC_STEPS):
                t_step = (t_cycle_start
                          + t_obs / 1000 + t_infer / 1000
                          + (step + 1) * step_dt)
                x, y, z, roll, pitch, yaw, g_norm = parse_step(
                    actions, step, ee_state
                )
                arm.move_to_ee(x, y, z, roll, pitch, yaw)
                gripper.set(g_norm * config.GRIPPER_MAX_POS)
                remaining = t_step - time.time()
                if remaining > 0:
                    time.sleep(remaining)
                elif remaining < -0.010:
                    logger.warning(f"    step {step} 超时 {-remaining*1000:.1f}ms")
            t_exec = (time.time() - t0) * 1000

            t_cycle = (time.time() - t_cycle_start) * 1000
            t_obs_list.append(t_obs)
            t_infer_list.append(t_infer)
            t_exec_list.append(t_exec)
            t_cycle_list.append(t_cycle)

            logger.info(
                f"[{cycle:4d}] obs={t_obs:5.1f}ms  infer={t_infer:6.1f}ms  "
                f"exec={t_exec:6.1f}ms  cycle={t_cycle:6.1f}ms  "
                f"({1000/t_cycle:.2f}Hz)  "
                f"ee=[{ee_state[0]:.3f},{ee_state[1]:.3f},{ee_state[2]:.3f}]"
            )

    except KeyboardInterrupt:
        logger.info("\nCtrl+C，正在安全退出 ...")

    finally:
        cams.close()
        gripper.close()
        arm.release()

        if t_cycle_list:
            print("\n" + "=" * 60)
            print(f"  统计汇总  {len(t_cycle_list)} cycles")
            print("=" * 60)
            for name, lst in [
                ("读观测  ", t_obs_list),
                ("GR00T推理", t_infer_list),
                (f"执行{config.EXEC_STEPS}步 ", t_exec_list),
                ("总cycle ", t_cycle_list),
            ]:
                arr = np.array(lst)
                print(f"  {name}: avg={arr.mean():.1f}ms  "
                      f"min={arr.min():.1f}  max={arr.max():.1f}  std={arr.std():.1f}")
            avg_cycle = np.mean(t_cycle_list)
            print(f"\n  机械臂控制频率: {1000/avg_cycle:.2f} Hz  "
                  f"（每 cycle 执行 {config.EXEC_STEPS} 步，推理+执行={avg_cycle:.0f}ms）")
            print("=" * 60)

        logger.info("程序结束")


if __name__ == "__main__":
    main()
