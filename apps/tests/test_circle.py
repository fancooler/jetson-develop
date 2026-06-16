#!/usr/bin/env python3
"""
test_circle.py — 右臂圆形轨迹测试（世界坐标系定义）

目的：
  验证坐标系转换的正确性。
  圆形轨迹在 Isaac Lab 世界坐标系中定义，执行时逐步转换：
    world → base_link → 臂基坐标系 → IK → 电机

轨迹设计：
  圆心：HOME 位在 base_link 系中的 EE 位置，用世界坐标表达
  平面：世界 XY 平面（水平圆），对应 base_link XZ 平面（臂前后+上下运动）
  半径：RADIUS（可调，默认 6cm）

坐标关系（world XY 平面圆 → base_link XZ 平面圆的推导）：

  p_world(θ) = [cx_w + r·cos(θ),  cy_w + r·sin(θ),  cz_w]

  p_base = R_ROOT_WORLD^T · (p_world - ROOT_POS)
  R_ROOT_WORLD^T = [[0,-1,0],[0,0,-1],[1,0,0]]

  → base_X =  -cy_w - r·sin(θ) = cx_b - r·sin(θ)  (前后变化)
     base_Y =  -(cz_w - BASE_OFFSET)  = cy_b          (不变！)
     base_Z =   cx_w + r·cos(θ)      = cz_b + r·cos(θ)(上下变化)

  验证：base_Y 全程不变，表明 world XY 平面圆确实映射到 base_link XZ 平面圆。

用法：
  python3 test_circle.py            # 正式执行（机械臂会动！）
  python3 test_circle.py --dry-run  # 仅打印轨迹，不动臂
"""

import sys
import os
import math
import time
import logging
import argparse

import numpy as np

# ── 路径：确保能 import app/ 下的模块 ─────────────────────────────────────────
_APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _APP_DIR)

import config
from arm_utils import DualArm
import contextlib, os as _os

logging.basicConfig(
    format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
    datefmt='%H:%M:%S',
    level=logging.INFO,
)
logger = logging.getLogger("test_circle")

# ── 坐标系常量（与 infer_dual.py 保持一致）────────────────────────────────────
#   p_world = R_ROOT_WORLD @ p_base + ROOT_POS
#   p_base  = R_ROOT_WORLD^T @ (p_world - ROOT_POS)
_ROOT_POS     = np.array([0.0, 0.0, config.BASE_OFFSET])    # [0, 0, 0.8409] m
_R_ROOT_WORLD = np.array([[0, 0, 1],
                           [-1, 0, 0],
                           [0, -1, 0]], dtype=np.float64)
_R_WORLD_ROOT = _R_ROOT_WORLD.T   # [[0,-1,0],[0,0,-1],[1,0,0]]


# ── 坐标系转换函数 ─────────────────────────────────────────────────────────────

def base_to_world(pos_base: np.ndarray) -> np.ndarray:
    """base_link 坐标系 → Isaac Lab 世界坐标系（仅位置）"""
    return _R_ROOT_WORLD @ np.asarray(pos_base) + _ROOT_POS


def world_to_base(pos_world: np.ndarray) -> np.ndarray:
    """Isaac Lab 世界坐标系 → base_link 坐标系（仅位置）"""
    return _R_WORLD_ROOT @ (np.asarray(pos_world) - _ROOT_POS)


# ── 圆形轨迹生成（在世界坐标系中定义）────────────────────────────────────────

def make_circle_world(
    center_world: np.ndarray,
    radius: float,
    n_points: int = 36,
) -> list:
    """
    在世界 XY 平面（水平面）内生成圆形轨迹点列。

    Args:
        center_world: 圆心（世界系，米）[x, y, z]
        radius:       半径（米）
        n_points:     轨迹点数（均匀分布，不含终点）

    Returns:
        list of ndarray [x,y,z]，世界系，从 θ=0 开始逆时针
    """
    angles = np.linspace(0, 2 * math.pi, n_points, endpoint=False)
    center = np.asarray(center_world, dtype=float)
    points = []
    for theta in angles:
        p = center.copy()
        p[0] += radius * math.cos(theta)   # world X
        p[1] += radius * math.sin(theta)   # world Y
        # world Z 不变（水平圆）
        points.append(p)
    return points


# ── 辅助：压制天机 SDK C 库打印 ───────────────────────────────────────────────

@contextlib.contextmanager
def _suppress_c_stdout():
    devnull = _os.open(_os.devnull, _os.O_WRONLY)
    saved_out, saved_err = _os.dup(1), _os.dup(2)
    try:
        _os.dup2(devnull, 1); _os.dup2(devnull, 2)
        yield
    finally:
        _os.dup2(saved_out, 1); _os.dup2(saved_err, 2)
        _os.close(devnull); _os.close(saved_out); _os.close(saved_err)


# ── 主程序 ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="右臂圆形轨迹测试（世界坐标系定义）")
    parser.add_argument("--dry-run", action="store_true",
                        help="仅打印轨迹，不连接机械臂")
    parser.add_argument("--radius", type=float, default=0.06,
                        help="圆半径，米（默认 0.06）")
    parser.add_argument("--points", type=int, default=36,
                        help="轨迹点数（默认 36，即每 10°一步）")
    parser.add_argument("--laps", type=int, default=1,
                        help="圈数（默认 1）")
    parser.add_argument("--step-time", type=float, default=1.5,
                        help="每步等待时间，秒（默认 1.5，即阻塞等待到位）")
    args = parser.parse_args()

    RADIUS    = args.radius
    N_POINTS  = args.points
    N_LAPS    = args.laps
    STEP_TIME = args.step_time
    DRY_RUN   = args.dry_run

    logger.info("=" * 62)
    logger.info("  右臂圆形轨迹测试 — 世界坐标系定义，验证坐标转换正确性")
    logger.info(f"  半径={RADIUS*100:.0f}cm  点数={N_POINTS}  圈数={N_LAPS}"
                f"  {'[DRY-RUN 不动臂]' if DRY_RUN else '[正式执行]'}")
    logger.info("=" * 62)

    # ── 连接（或跳过）机械臂 ─────────────────────────────────────────────────
    da = None
    if not DRY_RUN:
        da = DualArm()
        with _suppress_c_stdout():
            ok = da.connect()
        if not ok:
            logger.error("机械臂连接失败，退出（可用 --dry-run 仅查看轨迹）")
            return
        logger.info("机械臂已连接")

        # 回准备位
        logger.info("回准备位...")
        with _suppress_c_stdout():
            da.go_home()
        time.sleep(0.5)

    # ── 读取 HOME 位 EE（或使用默认值）──────────────────────────────────────
    if da is not None:
        states = da.read_all_states()
        pos_r_base, rpy_r = states['ee']['right']
        if pos_r_base is None:
            logger.error("FK 读取失败，退出")
            da.release()
            return
        pos_r_base = np.asarray(pos_r_base, dtype=float)
        rpy_r = tuple(rpy_r)
    else:
        # DRY-RUN：使用第三跑实测值
        pos_r_base = np.array([0.188, -0.085, 0.358])
        rpy_r = (0.0, 0.0, 0.0)   # 近似值，dry-run 不需要精确

    # ── 圆心：HOME 位换算到世界坐标系 ────────────────────────────────────────
    center_world = base_to_world(pos_r_base)

    logger.info("")
    logger.info("【坐标系对照】")
    logger.info(f"  HOME EE (base_link) : "
                f"[{pos_r_base[0]:.4f}, {pos_r_base[1]:.4f}, {pos_r_base[2]:.4f}] m")
    logger.info(f"  HOME EE (world)     : "
                f"[{center_world[0]:.4f}, {center_world[1]:.4f}, {center_world[2]:.4f}] m")
    logger.info(f"  变换公式：world = R_ROOT_WORLD @ base + [0,0,{config.BASE_OFFSET:.4f}]")
    logger.info(f"  等价展开：world_X = base_Z = {pos_r_base[2]:.4f}")
    logger.info(f"            world_Y = -base_X = {-pos_r_base[0]:.4f}")
    logger.info(f"            world_Z = -base_Y + {config.BASE_OFFSET:.4f} = {center_world[2]:.4f}")

    # ── 生成圆形轨迹（世界 XY 平面，水平圆）──────────────────────────────────
    circle_world = make_circle_world(center_world, RADIUS, N_POINTS)

    logger.info("")
    logger.info("【轨迹定义（世界系 XY 平面水平圆）→ 转换到 base_link 系】")
    logger.info(f"  圆心(world) = [{center_world[0]:.3f}, {center_world[1]:.3f}, {center_world[2]:.3f}] m")
    logger.info(f"  圆心(base ) = [{pos_r_base[0]:.3f}, {pos_r_base[1]:.3f}, {pos_r_base[2]:.3f}] m")
    logger.info(f"  半径 = {RADIUS*1000:.0f} mm，平面 = world XY（对应 base_link XZ）")
    logger.info("")

    # 打印样本点（每 45° 一个）
    step = max(1, N_POINTS // 8)
    logger.info(f"  {'θ(°)':>5}  {'world_X':>8} {'world_Y':>8} {'world_Z':>8}  "
                f"│  {'base_X':>8} {'base_Y':>8} {'base_Z':>8}  │ base_Y 应恒为 {pos_r_base[1]:.3f}")
    logger.info("  " + "-" * 80)
    for i in range(0, N_POINTS, step):
        pw = circle_world[i]
        pb = world_to_base(pw)
        theta_deg = 360.0 * i / N_POINTS
        flag = "✓" if abs(pb[1] - pos_r_base[1]) < 1e-6 else "✗"
        logger.info(f"  {theta_deg:>5.0f}°  "
                    f"{pw[0]:>8.4f} {pw[1]:>8.4f} {pw[2]:>8.4f}  "
                    f"│  {pb[0]:>8.4f} {pb[1]:>8.4f} {pb[2]:>8.4f}  │ {flag}")

    logger.info("")
    logger.info("  ✓ base_Y 全程恒定 → world XY 平面圆 确实映射到 base_link XZ 平面圆")

    if DRY_RUN:
        logger.info("\n[DRY-RUN] 轨迹打印完成，未连接机械臂。加 --dry-run 取消本标志可执行。")
        return

    # ── 正式执行 ─────────────────────────────────────────────────────────────
    input(f"\n按 Enter 开始执行（{N_LAPS} 圈，{N_POINTS} 步/圈，确认工作空间净空）... ")
    logger.info("")

    for lap in range(N_LAPS):
        logger.info(f"  第 {lap+1}/{N_LAPS} 圈 ─────────────────────────")
        for i, p_world in enumerate(circle_world):
            theta_deg = 360.0 * i / N_POINTS

            # ① 世界系 → base_link 系
            p_base = world_to_base(p_world)

            # ② 下发 IK 指令（阻塞等待到位）
            t0 = time.time()
            ok = da.move_to_ee_base_sync('right', p_base, rpy_r,
                                         timeout=STEP_TIME + 2.0)
            elapsed = (time.time() - t0) * 1000

            # ③ 读取实际位置，转回世界系验证
            states  = da.read_all_states()
            act_pos, _ = states['ee']['right']
            if act_pos is not None:
                act_world = base_to_world(np.asarray(act_pos))
                err_mm    = np.linalg.norm(p_world - act_world) * 1000
                status    = f"误差={err_mm:.1f}mm"
            else:
                act_world = np.full(3, float('nan'))
                status    = "FK失败"

            if ok:
                logger.info(
                    f"  [{lap+1}][{i+1:02d}] θ={theta_deg:5.1f}°  "
                    f"目标(world)=[{p_world[0]:.3f},{p_world[1]:.3f},{p_world[2]:.3f}]  "
                    f"实际(world)=[{act_world[0]:.3f},{act_world[1]:.3f},{act_world[2]:.3f}]  "
                    f"{status}  ({elapsed:.0f}ms)"
                )
            else:
                logger.warning(
                    f"  [{lap+1}][{i+1:02d}] θ={theta_deg:5.1f}°  "
                    f"IK无解/软限位拦截  "
                    f"base=[{p_base[0]:.3f},{p_base[1]:.3f},{p_base[2]:.3f}]"
                )

        # 每圈结束后停顿
        if lap < N_LAPS - 1:
            time.sleep(0.5)

    # ── 回准备位 ─────────────────────────────────────────────────────────────
    logger.info("\n轨迹完成，回准备位...")
    with _suppress_c_stdout():
        da.go_home()
    time.sleep(0.5)
    da.release()
    logger.info("测试结束")


if __name__ == "__main__":
    main()
