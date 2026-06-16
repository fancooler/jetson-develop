#!/usr/bin/env python3
"""
demo_right.py — 右臂 EE 控制验证（坐标系标定版）

目的：验证 frame_transform.py 是否正确，逐步建立 base_link ↔ 物理位置对应关系。

右臂工作约束（base_link 坐标系）：
  右臂 J0 安装在 base_link (X=0.617, Y=-0.372, Z=0.035) m
  可达条件：base_link_Y < -0.372 m（否则超出臂工作空间）
  臂坐标系距 J0 建议 < 620 mm（超出可能被控制器静默拒绝）

步骤：
  0. 打印初始状态（当前位置基准）
  1. 末端 → POS_A (0.40, -0.60, 0.52)  臂坐标系约 [485,217,228]mm，J0距离≈578mm
  2. 末端 → POS_B (0.45, -0.55, 0.50)  臂坐标系约 [465,167,178]mm，J0距离≈525mm（已验证）
  3. 回准备位

每步打印：base_link 坐标 + 臂坐标系 FK 原始值 + 关节角。

注：EE_RPY=(0,0,π/2) 经臂坐标系变换后 B=90°（ZYX 万向节死锁），
    但旋转矩阵保持正确（误差 <3e-8），frame_transform 已静默处理该警告。

用法：
    python3 demo_right.py          # 真实硬件
    python3 demo_right.py --mock   # Mock（无硬件）
"""

import sys
import os
import time
import math
import logging
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config_dual as config
from arm_utils import DualArm, MockDualArm
from frame_transform import base_to_fk

logging.basicConfig(
    format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
    datefmt='%H:%M:%S',
    level=logging.INFO,
)
logger = logging.getLogger('demo_right')

STEP_PAUSE  = 5.0
ARM_TIMEOUT = 30.0

# 目标点（base_link 系，米）— Y 必须 < -0.372（右臂工作空间约束）
#
# 臂坐标系换算（p_fk = R_ARM^T @ (p_base - T_RIGHT)）：
#   arm_X_mm = (base_Z - 0.035) × 1000
#   arm_Y_mm = -(base_X - 0.617) × 1000
#   arm_Z_mm = -(base_Y + 0.372) × 1000
#
# 已验证：
#   POS_B=[0.45,-0.55,0.50] → 臂坐标系 [465,167,178]mm，IK J2=-85.4°  ✓ 可达
#
# 根因：硬件 J2 实际限位约 -86°（代码软限位 -100° 太宽松）。
#   POS_A=[0.40,-0.55,0.50]（arm_Y=217mm）→ IK J2=-86.44°，超出硬件限位 → 咔哒不动
#   POS_A=[0.45,-0.55,0.50]（arm_Y=167mm）→ IK J2=-85.43°，勉强通过 → 改为此值
#
# 新策略：POS_A 保持与 POS_B 相同的 base_X（0.45m）和 base_Z（0.50m），
#          只改变 base_Y（更远离机体），这样 arm_Y 不变（167mm），
#          J2 与 POS_B 相近（安全），仅 arm_Z 不同（228mm vs 178mm）→ 验证 Z 方向坐标变换。
POS_A = [0.45, -0.60, 0.50]   # 臂坐标系约 [465, 167, 228] mm，距 J0≈544 mm
POS_B = [0.45, -0.55, 0.50]   # 臂坐标系约 [465, 167, 178] mm，距 J0≈525 mm（已验证）

# 姿态：与准备位保持一致，约 rpy=(0, 0, π/2)
EE_RPY = (0.0, 0.0, math.pi / 2)


def banner(n, desc):
    print()
    print('=' * 62)
    print(f'  步骤 {n}：{desc}')
    print('=' * 62)


def print_state(arm, label=''):
    """打印右臂完整状态，便于标定对比。"""
    states = arm.get_ee_states_base()
    joints = arm.read_joints()
    fk     = arm.get_fk_raw()

    pos, rpy = states['right']
    j        = joints['right']
    f        = fk['right']

    prefix = f'  [{label}] ' if label else '  '
    if pos is not None:
        print(f"{prefix}base_link: "
              f"pos=[{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}] m  "
              f"rpy=[{math.degrees(rpy[0]):+.1f}, "
              f"{math.degrees(rpy[1]):+.1f}, "
              f"{math.degrees(rpy[2]):+.1f}]°")
    else:
        print(f"{prefix}base_link: FK 失败")
    if f:
        print(f"{prefix}臂坐标系: "
              f"xyz=[{f[0]:.1f}, {f[1]:.1f}, {f[2]:.1f}] mm  "
              f"ABC=[{f[3]:.1f}, {f[4]:.1f}, {f[5]:.1f}]°")
    print(f"{prefix}joints=[{', '.join(f'{v:+.2f}' for v in j)}]°")


def move_and_pause(arm, pos, rpy, label, timeout):
    # ── 诊断：打印臂坐标系 IK 输入 ───────────────────────────────────────────
    xyzabc = base_to_fk(pos, rpy, 'right')
    x_mm, y_mm, z_mm = xyzabc[:3]
    a, b, c = xyzabc[3:]
    dist_mm = math.sqrt(x_mm**2 + y_mm**2 + z_mm**2)
    print(f'  IK输入: 臂坐标系 [{x_mm:.0f},{y_mm:.0f},{z_mm:.0f}]mm  '
          f'ABC=[{a:.1f},{b:.1f},{c:.1f}]°  J0距={dist_mm:.0f}mm')
    # ── 执行 ─────────────────────────────────────────────────────────────────
    ok = arm.move_to_ee_base_sync('right', pos, rpy, timeout)
    status = '✓ 到位' if ok else '✗ IK 无解或超时'
    print(f'  结果: {status}')
    print_state(arm, label)
    print(f'  暂停 {STEP_PAUSE:.0f}s ...')
    time.sleep(STEP_PAUSE)
    return ok


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mock', action='store_true')
    args = parser.parse_args()

    print()
    print('=' * 62)
    print('  右臂 EE 控制验证')
    print(f'  模式: {"Mock（无硬件）" if args.mock else "真实硬件"}')
    print('=' * 62)

    if args.mock:
        arm = MockDualArm()
        arm.connect()
    else:
        arm = DualArm()
        if not arm.connect():
            logger.error('连接失败')
            return

    # ── 步骤 0：打印初始状态 ──────────────────────────────────────────────────
    banner(0, '初始状态（准备位基准）')
    print_state(arm, '初始')
    print(f'\n  目标 POS_A = {POS_A}')
    print(f'  目标 POS_B = {POS_B}')
    print(f'  姿态 EE_RPY = (0°, 0°, 90°)')
    time.sleep(2.0)

    try:
        # ── 步骤 1：→ POS_A ───────────────────────────────────────────────────
        banner(1, f'末端 → POS_A {POS_A} m')
        move_and_pause(arm, POS_A, EE_RPY, '到达后', ARM_TIMEOUT)

        # ── 步骤 2：→ POS_B ───────────────────────────────────────────────────
        banner(2, f'末端 → POS_B {POS_B} m')
        move_and_pause(arm, POS_B, EE_RPY, '到达后', ARM_TIMEOUT)

        # ── 步骤 3：回准备位 ──────────────────────────────────────────────────
        banner(3, '回准备位')
        arm.go_home(arm='right')
        print_state(arm, '回位后')

        print()
        print('=' * 62)
        print('  完成 ✓')
        print('=' * 62)

    except KeyboardInterrupt:
        print('\nCtrl+C ...')

    finally:
        arm.go_home(arm='right')
        arm.release()
        logger.info('已退出')


if __name__ == '__main__':
    main()
