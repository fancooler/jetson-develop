#!/usr/bin/env python3
"""
test/test_joint_limits.py — 实测关节软限位（上机验证脚本）

测试原理：
  从 home 出发，逐步向负方向命令更大的关节角（绕过 Python 软限位检查），
  等待机械臂响应后读回实际角度。若实际角与目标角差 > 1°，判定为硬件拒绝
  （咔哒声 / 静默不动），记录边界。

测试关节：
  J2（索引1）、J5（索引4）负方向限位
  说明：J2/J5 是唯一与 SDK 标称限位差异显著的关节，也是当前运行中 IK 被拦截的根因。

运行方式（在 Jetson 上，需连接机械臂）：
    cd ~/work/app
    python3 test/test_joint_limits.py               # 测右臂（默认）
    python3 test/test_joint_limits.py --arm left    # 测左臂
    python3 test/test_joint_limits.py --arm both    # 双臂都测
    python3 test/test_joint_limits.py --dry-run     # 预览测试步骤，不动臂

安全设计：
  - 每步前先回 home，确保从已知安全姿态出发
  - 连续 2 次未到达则停止扫描，避免反复撞限位
  - 测试范围最大到 -93°（SDK 允许 -120°，此处保守截断）
  - 步长 0.5°，精度足够，不会在限位附近猛冲
"""

import sys
import os
import time
import argparse
import contextlib

import numpy as np

# ── 路径设置（从 test/ 子目录运行时将 app/ 加入 path）──────────────────────────
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config_dual as config
from arm_utils import SingleArm, _JOINT_LIMITS_DEG


# ── 测试参数 ──────────────────────────────────────────────────────────────────

# 各关节扫描配置（负方向）
# 从 -80° 开始（远离 home 的 -61°，避免大量无意义步骤），扫到 -93°
_SWEEP = {
    'J2': {'idx': 1, 'start': -80.0, 'stop': -93.0, 'step': -0.5},
    'J5': {'idx': 4, 'start': -80.0, 'stop': -93.0, 'step': -0.5},
}

_REACH_TOL_DEG   = 1.0   # 实际角与目标角之差 < 1° 视为"到达"
_MOVE_TIMEOUT_S  = 6.0   # 每步运动超时（含从 home 出发的大幅运动）
_SETTLE_S        = 0.3   # 运动结束后额外等待时间
_FAIL_STOP_COUNT = 2     # 连续几次失败后停止扫描


# ── C 库输出抑制 ──────────────────────────────────────────────────────────────

@contextlib.contextmanager
def _suppress_c():
    """屏蔽天机 SDK C 库的 stdout/stderr。"""
    devnull = os.open(os.devnull, os.O_WRONLY)
    saved_out, saved_err = os.dup(1), os.dup(2)
    try:
        os.dup2(devnull, 1)
        os.dup2(devnull, 2)
        yield
    finally:
        os.dup2(saved_out, 1)
        os.dup2(saved_err, 2)
        os.close(devnull)
        os.close(saved_out)
        os.close(saved_err)


# ── 运动辅助 ─────────────────────────────────────────────────────────────────

def _go_home_blocking(arm: SingleArm):
    """回 home 并等待到位。"""
    with _suppress_c():
        arm.go_home()
    time.sleep(0.8)


def _move_joints_and_readback(arm: SingleArm, test_joints: list) -> list:
    """
    发关节角指令（bypass 软限位），等待响应后读回实际角度。

    move_joints_sync 内部直接调用 set_joint_position_cmd，不检查 _JOINT_LIMITS_DEG。
    若机械臂咔哒/不动，_MOVE_TIMEOUT_S 后超时返回 False，实际角度不变。
    """
    with _suppress_c():
        arm.move_joints_sync(test_joints, timeout=_MOVE_TIMEOUT_S)
    time.sleep(_SETTLE_S)
    return arm.read_joints()


# ── 单关节扫描 ────────────────────────────────────────────────────────────────

def sweep_joint(arm: SingleArm, arm_str: str,
                joint_name: str, sweep_cfg: dict,
                home_joints: list, dry_run: bool) -> dict:
    """
    从 start 向 stop 方向扫描，逐步命令关节角，记录实际限位。

    Returns:
        {
          'last_reached': float | None,   # 最后一个成功到达的角度
          'first_failed': float | None,   # 第一个失败的角度
        }
    """
    idx   = sweep_cfg['idx']
    start = sweep_cfg['start']
    stop  = sweep_cfg['stop']
    step  = sweep_cfg['step']

    soft_lo, soft_hi = _JOINT_LIMITS_DEG[idx]
    angles = np.arange(start, stop + step / 2, step)

    print(f"\n  ── {joint_name}（索引{idx}）负方向扫描 ──")
    print(f"     Python 软限位 : {soft_lo:.1f}°")
    print(f"     SDK 配置限位  : -120.0°（J2）/ -170.0°（J5）")
    print(f"     扫描范围      : {start:.1f}° → {stop:.1f}°，步长 {abs(step):.1f}°")
    print(f"     到达判定阈值  : 误差 < {_REACH_TOL_DEG}°")
    print()

    if dry_run:
        for angle in angles:
            print(f"     [DRY] {joint_name} = {angle:+.1f}°  （home → 目标 → home）")
        return {'last_reached': None, 'first_failed': None}

    last_reached = None
    first_failed = None
    fail_count   = 0

    for angle in angles:
        angle = round(float(angle), 2)

        # 构造测试关节角：只改当前关节，其余保持 home
        test_joints = list(home_joints)
        test_joints[idx] = angle

        # 每步先回 home，确保初始姿态一致
        _go_home_blocking(arm)

        # 发指令，读回
        actual = _move_joints_and_readback(arm, test_joints)
        actual_angle = actual[idx]
        err = abs(actual_angle - angle)
        reached = (err < _REACH_TOL_DEG)

        marker = "✓" if reached else "✗"
        note   = f"误差 {err:.1f}°" if not reached else f"误差 {err:.2f}°"
        beyond_soft = " ← 超出软限位" if angle < soft_lo else ""
        print(f"     {marker} {joint_name} = {angle:+.1f}°  "
              f"实际 = {actual_angle:+.1f}°  {note}{beyond_soft}")

        if reached:
            last_reached = angle
            fail_count = 0
        else:
            if first_failed is None:
                first_failed = angle
            fail_count += 1
            if fail_count >= _FAIL_STOP_COUNT:
                print(f"     → 连续 {fail_count} 次未到达，停止扫描")
                break

    return {'last_reached': last_reached, 'first_failed': first_failed}


# ── 单臂测试 ──────────────────────────────────────────────────────────────────

def test_arm(arm_str: str, dry_run: bool):
    """测试单臂 J2、J5 负方向限位。"""
    print(f"\n{'='*60}")
    print(f"  {arm_str.upper()} 臂关节限位实测")
    print(f"{'='*60}")

    arm = SingleArm(arm_str)

    if not dry_run:
        with _suppress_c():
            ok = arm.connect()
        if not ok:
            print(f"  [✗] 连接失败，跳过 {arm_str} 臂")
            return

    home = (config.HOME_JOINTS_LEFT  if arm_str == 'left'
            else config.HOME_JOINTS_RIGHT)
    print(f"  Home 关节角: {[f'{j:.1f}' for j in home]}")

    if not dry_run:
        print("  回 home ...")
        _go_home_blocking(arm)

    # 测试 J2 和 J5
    results = {}
    for jname, cfg_sweep in _SWEEP.items():
        results[jname] = sweep_joint(
            arm, arm_str, jname, cfg_sweep, home, dry_run
        )

    # ── 汇总报告 ──────────────────────────────────────────────────────────────
    print(f"\n  ── {arm_str.upper()} 臂 汇总 ──")
    print(f"  {'关节':<5} {'最后到达':>10} {'首次失败':>10} {'当前软限位':>12}  结论")
    print(f"  {'-'*55}")
    for jname, r in results.items():
        idx  = _SWEEP[jname]['idx']
        soft = _JOINT_LIMITS_DEG[idx][0]
        last  = f"{r['last_reached']:+.1f}°" if r['last_reached'] is not None else "  N/A"
        first = f"{r['first_failed']:+.1f}°" if r['first_failed'] is not None else "  N/A"

        if r['last_reached'] is not None and r['first_failed'] is not None:
            margin = r['last_reached'] - soft   # 负数 = 软限位偏保守
            if margin < -0.1:
                conclusion = f"软限位偏保守，实际还能走 {abs(margin):.1f}°"
            elif margin > 0.1:
                conclusion = f"⚠ 软限位偏激进（实际 {r['last_reached']:.1f}° 已超硬限）"
            else:
                conclusion = "软限位与硬件基本吻合"
        else:
            conclusion = "数据不足"

        print(f"  {jname:<5} {last:>10} {first:>10} {soft:>+10.1f}°  {conclusion}")

    if not dry_run:
        print("\n  回 home ...")
        _go_home_blocking(arm)
        arm.release()
        print(f"  {arm_str} 臂已释放")


# ── 入口 ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="实测关节软限位（需在 Jetson 上连接机械臂运行）"
    )
    parser.add_argument(
        '--arm', choices=['right', 'left', 'both'], default='right',
        help="测试哪条臂（默认 right）"
    )
    parser.add_argument(
        '--dry-run', action='store_true',
        help="仅打印测试步骤，不实际下发运动指令"
    )
    args = parser.parse_args()

    arms = ['right', 'left'] if args.arm == 'both' else [args.arm]

    print("=" * 60)
    print("  关节软限位实测  (J2 / J5 负方向)")
    print("  每步前回 home，发现连续 2 次失败即停")
    if args.dry_run:
        print("  [DRY RUN] 仅打印计划，不下发指令")
    print("=" * 60)

    for arm_str in arms:
        test_arm(arm_str, dry_run=args.dry_run)

    print("\n" + "=" * 60)
    print("  全部测试完成")
    print("=" * 60)


if __name__ == "__main__":
    main()
