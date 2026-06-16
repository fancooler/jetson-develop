#!/usr/bin/env python3
"""
时序测试脚本

用 MockPolicy 替代 GR00T 推理（在当前位置附近小幅正弦振荡），
其余完全走真实流程：摄像头读帧、臂状态读取、IK、发关节指令。

输出每个 cycle 的时间分解，以及 N 个 cycle 后的统计汇总。

运行方式：
    python3 test_timing.py [cycles]     # 默认跑 20 个 cycle
"""
import sys
import os
import time
import math
import logging
import numpy as np

logging.basicConfig(
    format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
    datefmt='%H:%M:%S',
    level=logging.WARNING,   # 压制 SDK 日志，只看计时输出
)
logger = logging.getLogger("timing")
logging.getLogger("arm").setLevel(logging.WARNING)
logging.getLogger("camera").setLevel(logging.WARNING)

sys.path.insert(0, os.path.dirname(__file__))
import config

from arm     import TJArm
from gripper import MockGripper
from camera  import DualCamera

MAX_CYCLES = int(sys.argv[1]) if len(sys.argv) > 1 else 20

# ── MockPolicy ────────────────────────────────────────────────────────────────

class MockPolicy:
    """
    在当前末端位置附近做小幅正弦振荡，模拟 16 步动作序列。
    X 轴 ±5mm、Z 轴 ±5mm，不改变姿态。
    """
    def __init__(self):
        self._t = 0.0

    def get_action(self, obs: dict) -> dict:
        # 从 obs 中取当前末端状态（已是 m/rad）
        x0 = float(obs["state.x"][0, 0])
        y0 = float(obs["state.y"][0, 0])
        z0 = float(obs["state.z"][0, 0])
        r0 = float(obs["state.roll"][0, 0])
        p0 = float(obs["state.pitch"][0, 0])
        yw = float(obs["state.yaw"][0, 0])

        dt   = 1.0 / config.EXEC_HZ
        amp  = 0.005   # 5mm 振幅

        actions = {k: [] for k in
                   ["action.x","action.y","action.z",
                    "action.roll","action.pitch","action.yaw","action.gripper"]}

        for i in range(config.ACTION_HORIZON):
            t = self._t + i * dt
            actions["action.x"].append(x0 + amp * math.sin(2 * math.pi * 0.5 * t))
            actions["action.y"].append(y0)
            actions["action.z"].append(z0 + amp * math.sin(2 * math.pi * 0.3 * t))
            actions["action.roll"].append(r0)
            actions["action.pitch"].append(p0)
            actions["action.yaw"].append(yw)
            actions["action.gripper"].append(0.5)

        self._t += config.ACTION_HORIZON * dt
        return {k: np.array(v) for k, v in actions.items()}


# ── 辅助 ──────────────────────────────────────────────────────────────────────

def parse_step(actions, step):
    def _g(k): return float(np.atleast_1d(actions[k])[step])
    ag = np.atleast_1d(actions["action.gripper"])
    g  = float(ag[step]) * config.GRIPPER_MAX_POS
    return _g("action.x"), _g("action.y"), _g("action.z"), \
           _g("action.roll"), _g("action.pitch"), _g("action.yaw"), g


# ── 主函数 ────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print(f"  时序测试  MockPolicy  {MAX_CYCLES} cycles")
    print("=" * 60)

    arm     = TJArm()
    gripper = MockGripper()
    cams    = DualCamera()

    if not arm.connect():
        print("机械臂连接失败")
        return

    cams.start()
    policy  = MockPolicy()

    # 回到准备位置
    print("运动到准备位置 ...")
    arm.go_home()
    time.sleep(0.5)

    step_dt = 1.0 / config.EXEC_HZ

    # 计时记录
    t_obs_list    = []   # 读观测耗时
    t_infer_list  = []   # 推理耗时
    t_exec_list   = []   # 执行 16 步耗时
    t_cycle_list  = []   # 总 cycle 耗时
    ik_fail_steps = 0
    total_steps   = 0

    print(f"\n{'Cycle':>5}  {'Obs ms':>7}  {'Infer ms':>9}  "
          f"{'Exec ms':>8}  {'Cycle ms':>9}  {'IK失败':>6}")
    print("-" * 60)

    try:
        for cycle in range(1, MAX_CYCLES + 1):
            t_cycle_start = time.time()

            # ── 读观测 ──────────────────────────────────────────────────────
            t0 = time.time()
            head_rgb, wrist_rgb = cams.read()
            ee_state            = arm.get_ee_state()
            gripper_pos         = gripper.query()[0]
            t_obs = (time.time() - t0) * 1000

            # 构造 obs
            x, y, z, roll, pitch, yaw = ee_state
            g_frac = gripper_pos / config.GRIPPER_MAX_POS
            obs = {
                "state.x":       np.array([[x]],               dtype=np.float32),
                "state.y":       np.array([[y]],               dtype=np.float32),
                "state.z":       np.array([[z]],               dtype=np.float32),
                "state.roll":    np.array([[roll]],            dtype=np.float32),
                "state.pitch":   np.array([[pitch]],           dtype=np.float32),
                "state.yaw":     np.array([[yaw]],             dtype=np.float32),
                "state.gripper": np.array([[g_frac*0.032, -g_frac*0.032]], dtype=np.float32),
                "annotation.human.action.task_description": [config.TASK],
            }

            # ── MockPolicy 推理 ──────────────────────────────────────────────
            t0 = time.time()
            actions = policy.get_action(obs)
            t_infer = (time.time() - t0) * 1000

            # ── 执行 16 步 ───────────────────────────────────────────────────
            t_exec_start = time.time()
            cycle_ik_fail = 0

            for step in range(config.ACTION_HORIZON):
                t_step = t_cycle_start + \
                         (t_obs + t_infer) / 1000.0 + (step + 1) * step_dt

                x_, y_, z_, r_, p_, yw_, gp = parse_step(actions, step)
                ok = arm.move_to_ee(x_, y_, z_, r_, p_, yw_)
                gripper.set(gp)
                total_steps += 1
                if not ok:
                    cycle_ik_fail += 1
                    ik_fail_steps += 1

                remaining = t_step - time.time()
                if remaining > 0:
                    time.sleep(remaining)

            t_exec  = (time.time() - t_exec_start) * 1000
            t_cycle = (time.time() - t_cycle_start) * 1000

            t_obs_list.append(t_obs)
            t_infer_list.append(t_infer)
            t_exec_list.append(t_exec)
            t_cycle_list.append(t_cycle)

            print(f"{cycle:>5}  {t_obs:>7.1f}  {t_infer:>9.1f}  "
                  f"{t_exec:>8.1f}  {t_cycle:>9.1f}  {cycle_ik_fail:>6}")

    except KeyboardInterrupt:
        print("\n用户中断")

    finally:
        cams.close()
        gripper.close()
        arm.release()

    # ── 汇总统计 ─────────────────────────────────────────────────────────────
    if not t_cycle_list:
        return

    def stats(lst, label):
        a = np.array(lst)
        print(f"  {label:<12}: avg={a.mean():.1f}ms  "
              f"min={a.min():.1f}ms  max={a.max():.1f}ms  "
              f"std={a.std():.1f}ms")

    n = len(t_cycle_list)
    print("\n" + "=" * 60)
    print(f"  统计汇总（{n} cycles，{total_steps} steps）")
    print("=" * 60)
    stats(t_obs_list,   "读观测")
    stats(t_infer_list, "Mock推理")
    stats(t_exec_list,  "执行16步")
    stats(t_cycle_list, "总cycle")

    avg_cycle  = np.mean(t_cycle_list)
    avg_exec   = np.mean(t_exec_list)
    infer_hz   = 1000.0 / avg_cycle
    exec_hz    = (config.ACTION_HORIZON * 1000.0) / avg_exec
    ik_rate    = 100.0 * ik_fail_steps / max(total_steps, 1)

    print(f"\n  推理频率  : {infer_hz:.2f} Hz  （期望: ~{1000/(config.ACTION_HORIZON/config.EXEC_HZ*1000 + 250):.2f} Hz）")
    print(f"  动作执行  : {exec_hz:.2f} Hz  （期望: {config.EXEC_HZ} Hz）")
    print(f"  IK失败率  : {ik_rate:.1f}%  ({ik_fail_steps}/{total_steps} steps)")
    print("=" * 60)


if __name__ == "__main__":
    main()
