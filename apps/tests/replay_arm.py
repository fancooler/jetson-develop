#!/usr/bin/env python3
"""
replay_arm.py — 单臂关节轨迹回放（CSV → 真机，左/右臂通用）

把一条「每行一帧、每列一个关节角(度)」的轨迹 CSV，按固定频率流式下发到指定臂，
用于在真机上回放训练轨迹、肉眼对比 sim/real 动作。

轨迹文件约定：
  - 无表头，N 行 × 7 列，单位「度」，列顺序 = J1..J7（与 arm_utils 一致）。
  - 默认原样下发（视为已是 SDK 约定）。
  - 若 CSV 是模型 action（URDF/模型约定，如 right_arm_action_deg.csv），加 --map
    经 joint_map.urdf_to_sdk 换算到 SDK 约定再下发（需先 calib_joints.py --save）。
    这是上机验证关节映射是否正确的首选手段：单臂、慢速、可逐帧肉眼对比 sim/real。
  - 本脚本不做左右镜像；右臂数据放左臂仅在确知镜像关系时才有意义。
    （见 memory: sim-real-coord-mismatch）

安全设计：
  - safe=True：每帧先过软限位检查；遇到第一帧超限即停止回放（不强行下发）。
  - 开播前先 move_joints_sync 慢速对准第 0 帧，避免「瞬移」。
  - 运动前需操作员按 Enter 确认（--yes 跳过）。
  - try/finally：无论正常结束/Ctrl+C/异常，都 go_home(arm) 再 release()，
    让该臂先回到 HOME 再下电，避免从悬停姿态失力下坠。
  - 只控制 --arm 指定的那条臂，另一条臂保持原位（注意潜在碰撞，现场盯紧）。

用法（在 Jetson ~/work/app 下运行）：
    cd ~/work/app
    python3 test/replay_arm.py --arm right                 # 右臂, 2Hz, 10%, 全程
    python3 test/replay_arm.py --arm left --steps 200       # 左臂, 仅前 200 步
    python3 test/replay_arm.py --arm left traj.csv --hz 2 --vel 10
    python3 test/replay_arm.py --arm left --steps 200 --mock # 无硬件，仅验证逻辑/时序
"""

import os
import sys
import time
import argparse
import logging

import numpy as np

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP_DIR)

import config_dual as config
import joint_map
from arm_utils import DualArm, MockDualArm, check_joints_in_limits, _JOINT_LIMITS_DEG

logging.basicConfig(
    format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
    datefmt='%H:%M:%S',
    level=logging.INFO,
)
logger = logging.getLogger('replay_arm')

DEFAULT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'right_arm_action_deg.csv')


def load_trajectory(path: str) -> np.ndarray:
    """读取 CSV → (N,7) float 数组（度）。校验列数与基本数值合法性。"""
    if not os.path.exists(path):
        logger.error(f"轨迹文件不存在: {path}")
        sys.exit(1)
    a = np.loadtxt(path, delimiter=',', ndmin=2)
    if a.shape[1] != 7:
        logger.error(f"列数应为 7（J1..J7），实为 {a.shape[1]}")
        sys.exit(1)
    if not np.isfinite(a).all():
        logger.error("轨迹含 NaN/Inf，已拒绝")
        sys.exit(1)
    return a


def precheck(traj: np.ndarray):
    """回放前打印轨迹概况 + 逐关节越限统计，返回首个越限帧索引（无则 len）。"""
    n = len(traj)
    logger.info(f"轨迹: {n} 帧 × 7 关节（度）")
    first_bad = n
    for i in range(n):
        if not check_joints_in_limits(list(traj[i])):
            first_bad = i
            break
    # 逐关节越限统计（仅提示用）
    for j in range(7):
        lo, hi = _JOINT_LIMITS_DEG[j]
        c = traj[:, j]
        n_over = int(((c < lo) | (c > hi)).sum())
        tag = f"  <-- {n_over}帧越限[{lo},{hi}]" if n_over else ""
        logger.info(f"  J{j+1}: min={c.min():8.2f} max={c.max():8.2f}{tag}")
    if first_bad < n:
        logger.warning(
            f"第 {first_bad} 帧起超软限位（safe=True 将在此停止回放）")
    return first_bad


def confirm(prompt: str):
    try:
        ans = input(prompt)
    except EOFError:
        ans = ''
    if ans.strip().lower() not in ('', 'y', 'yes'):
        logger.info("已取消")
        sys.exit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='?', default=DEFAULT_CSV, help='轨迹 CSV 路径')
    ap.add_argument('--arm', choices=['left', 'right'], default='right',
                    help='执行臂（默认 right）')
    ap.add_argument('--steps', type=int, default=None,
                    help='最多执行前 N 步（默认全部）')
    ap.add_argument('--hz', type=float, default=2.0, help='回放频率（默认 2Hz）')
    ap.add_argument('--vel', type=int, default=10, help='速度比率 %%（默认 10）')
    ap.add_argument('--acc', type=int, default=None, help='加速度比率 %%（默认=vel）')
    ap.add_argument('--start-timeout', type=float, default=60.0,
                    help='对准第 0 帧的阻塞超时（秒）')
    ap.add_argument('--mock', action='store_true', help='无硬件，仅验证逻辑/时序')
    ap.add_argument('--yes', action='store_true', help='跳过运动前确认')
    ap.add_argument('--map', action='store_true',
                    help='把 CSV 视为 URDF/模型动作，经 joint_map 换算到 SDK 约定再下发'
                         '（CSV 来自模型 action 时应开启；需先 calib_joints.py --save）')
    args = ap.parse_args()

    arm = args.arm
    acc = args.acc if args.acc is not None else args.vel
    period = 1.0 / args.hz

    # 运行时覆盖速度/加速度比率（arm_utils 每次都从 config_dual 动态读取）
    config.VEL_RATIO = args.vel
    config.ACC_RATIO = acc
    config.HOME_VEL_RATIO = args.vel

    traj = load_trajectory(args.csv)

    # ── 关节空间映射（URDF/模型 → SDK）──────────────────────────────────────────
    # CSV 是模型 action（URDF 约定）。开 --map 后整条轨迹先换算到 SDK 约定，
    # 之后的软限位检查 / 对准 / 下发全部在 SDK 空间，保持一致。
    if args.map:
        if not joint_map.is_active():
            logger.error(
                "--map 已开启但 joint_map 未生效（缺 test/joint_map.json 或 "
                "config_dual.USE_JOINT_MAP=False）。请先在 Jetson 跑 "
                "`python3 test/calib_joints.py --save`，或去掉 --map 原样回放。")
            sys.exit(1)
        traj = np.array([joint_map.urdf_to_sdk(arm, row) for row in traj])
        logger.info(f"已对 {len(traj)} 帧应用 joint_map.urdf_to_sdk('{arm}') → SDK 约定")
    else:
        logger.info("未开 --map：CSV 原样下发（视为已是 SDK 约定）")

    # 执行帧数 = min(轨迹长度, --steps)；越限会在循环里再提前截断
    play_n = len(traj) if args.steps is None else min(args.steps, len(traj))

    print()
    print('=' * 64)
    print(f'  单臂轨迹回放  ——  {arm.upper()} 臂')
    print(f'  文件: {args.csv}')
    print(f'  频率: {args.hz}Hz（{period*1000:.0f}ms/帧）   速度比率: {args.vel}%   acc: {acc}%')
    print(f'  步数: 前 {play_n} 帧 / 共 {len(traj)} 帧')
    print(f'  模式: {"Mock（无硬件）" if args.mock else "真实硬件"}')
    print(f'  映射: {"URDF→SDK (joint_map)" if args.map else "原样（无映射）"}')
    print('=' * 64)

    if 'right' in os.path.basename(args.csv) and arm == 'left':
        logger.warning("注意：right_arm 轨迹原样下发到【左臂】，未做左右镜像/符号翻转。")

    first_bad = precheck(traj)
    n_exec = min(first_bad, play_n)
    est_sec = n_exec * period
    logger.info(f"将执行 {n_exec} 帧，预计 ~{est_sec:.0f}s（不含对准起点时间）")

    # ── 连接 ─────────────────────────────────────────────────────────────────
    if args.mock:
        robot = MockDualArm()
        robot.connect()
    else:
        robot = DualArm()
        if not robot.connect():
            logger.error('连接失败')
            return

    try:
        # ── 进位置跟随模式（servo_reset 清残留 + 设 vel/acc）─────────────────
        if not args.mock:
            robot.enter_position_mode(arm)

        frame0 = list(traj[0])
        logger.info(f"[{arm}] 对准第 0 帧: [{', '.join(f'{v:.1f}' for v in frame0)}]°")
        if not args.yes:
            confirm(f"\n>>> 【{arm.upper()}臂】将先慢速移动到轨迹起点。"
                    f"确认现场安全后按 Enter 开始（其它键取消）: ")

        ok = robot.move_joints_sync(arm, frame0, timeout=args.start_timeout)
        if not ok:
            logger.error("对准第 0 帧失败/超时，中止回放")
            return
        logger.info("已到起点")

        if not args.yes:
            confirm(f">>> 开始 {args.hz}Hz 回放共 {n_exec} 帧（~{est_sec:.0f}s）。按 Enter 开始: ")

        # ── 流式回放 ──────────────────────────────────────────────────────────
        logger.info("回放开始 ...")
        t0 = time.monotonic()
        n_ok = n_sdk_fail = 0
        sent = 0
        for i in range(play_n):
            frame = list(traj[i])
            if not check_joints_in_limits(frame):
                logger.warning(
                    f"帧 {i}: 超软限位（J1={frame[0]:.1f}°），按设定停止回放")
                break

            if robot.move_joints(arm, frame, safe=True):
                n_ok += 1
            else:
                n_sdk_fail += 1
                logger.warning(f"帧 {i}: move_joints 返回 False（SDK/模式问题）")
            sent += 1

            if i % 20 == 0 or i == n_exec - 1:
                logger.info(f"  帧 {i:>3}/{n_exec}  J1={frame[0]:7.2f}°  "
                            f"J2={frame[1]:7.2f}°  J5={frame[4]:7.2f}°")

            # 单调时钟对齐，防累计漂移
            next_t = t0 + (i + 1) * period
            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)

        elapsed = time.monotonic() - t0
        eff_hz = sent / elapsed if elapsed > 0 else 0.0
        print()
        print('=' * 64)
        logger.info(f"回放结束: 下发 {sent} 帧（成功 {n_ok}，SDK失败 {n_sdk_fail}）")
        logger.info(f"耗时 {elapsed:.1f}s，实际频率 {eff_hz:.2f}Hz（目标 {args.hz}Hz）")
        print('=' * 64)

    except KeyboardInterrupt:
        print('\nCtrl+C，停止回放 ...')

    finally:
        logger.info(f"[{arm}] 回 HOME 并下电 ...")
        try:
            robot.go_home(arm=arm)
        except Exception as e:
            logger.warning(f"go_home 异常（继续 release）: {e}")
        robot.release()
        logger.info("已退出")


if __name__ == '__main__':
    main()
