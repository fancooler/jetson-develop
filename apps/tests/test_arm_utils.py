#!/usr/bin/env python3
"""
test/test_arm_utils.py — arm_utils 工具函数测试

【不依赖硬件】纯工具函数部分可在本机直接运行：
    python3 test/test_arm_utils.py

【依赖硬件】DualArm 集成测试需在 Jetson 上运行（--hw 参数）：
    python3 test/test_arm_utils.py --hw
"""

import sys
import os
import math
import argparse
import numpy as np

# 将 app/ 加入 path（从 test/ 子目录运行时）
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm_utils import (
    check_joints_in_limits,
    clamp_delta_pos,
    clamp_delta_rpy,
    apply_ee_delta,
    _JOINT_LIMITS_DEG,
)
from frame_transform import fk_to_base, base_to_fk

PASS = "✓"
FAIL = "✗"


# ═══════════════════════════════════════════════════════════════════════════════
# 纯工具函数测试（无硬件依赖）
# ═══════════════════════════════════════════════════════════════════════════════

def _check(name: str, cond: bool, detail: str = ""):
    status = PASS if cond else FAIL
    msg = f"  [{status}] {name}"
    if detail:
        msg += f"  ({detail})"
    print(msg)
    return cond


def test_joint_limits():
    print("\n=== check_joints_in_limits ===")
    all_ok = True

    # 全零在限位内
    joints_ok = [0.0] * 7
    all_ok &= _check("全零关节角在限位内", check_joints_in_limits(joints_ok))

    # J1 超出
    joints_bad = [0.0] * 7
    joints_bad[0] = 175.0   # J1 超出 (-170, 170)
    all_ok &= _check("J1=175° 应超限", not check_joints_in_limits(joints_bad))

    # J4 超出
    joints_bad2 = [0.0] * 7
    joints_bad2[3] = 70.0   # J4 超出 (-145, 60)
    all_ok &= _check("J4=70° 应超限", not check_joints_in_limits(joints_bad2))

    # 准备位关节角（应在限位内）
    try:
        import config_dual as config
        for name, home in [("LEFT HOME",  config.HOME_JOINTS_LEFT),
                            ("RIGHT HOME", config.HOME_JOINTS_RIGHT)]:
            all_ok &= _check(f"{name} 在限位内",
                              check_joints_in_limits(home),
                              str([f"{j:.1f}" for j in home]))
    except ImportError:
        print("  [跳过] 无法导入 config（HOME_JOINTS 检查）")

    return all_ok


def test_clamp_delta_pos():
    print("\n=== clamp_delta_pos ===")
    all_ok = True

    # 小增量不变
    small = np.array([0.01, 0.01, 0.01])
    clamped = clamp_delta_pos(small)
    all_ok &= _check("小增量不变", np.allclose(clamped, small),
                     f"in={small}, out={clamped}")

    # 大增量等比缩放
    big = np.array([0.1, 0.0, 0.0])
    clamped = clamp_delta_pos(big, max_step_m=0.05)
    all_ok &= _check("大增量缩放到 50mm",
                     abs(np.linalg.norm(clamped) - 0.05) < 1e-9,
                     f"norm={np.linalg.norm(clamped):.4f}m")

    # 方向保持
    diag = np.array([0.06, 0.06, 0.06])
    clamped = clamp_delta_pos(diag, max_step_m=0.05)
    cosine = np.dot(diag, clamped) / (np.linalg.norm(diag) * np.linalg.norm(clamped))
    all_ok &= _check("方向保持不变", abs(cosine - 1.0) < 1e-9,
                     f"cos={cosine:.6f}")

    return all_ok


def test_clamp_delta_rpy():
    print("\n=== clamp_delta_rpy ===")
    all_ok = True

    # 小增量不变
    small = np.array([0.1, 0.1, 0.1])
    clamped = clamp_delta_rpy(small)
    all_ok &= _check("小增量不变", np.allclose(clamped, small))

    # 大增量被截断
    big = np.array([1.0, -1.0, 0.5])
    clamped = clamp_delta_rpy(big, max_step_rad=0.30)
    all_ok &= _check("大增量截断到 ±0.30",
                     all(abs(v) <= 0.30 + 1e-9 for v in clamped),
                     str([f"{v:.3f}" for v in clamped]))
    return all_ok


def test_apply_ee_delta():
    print("\n=== apply_ee_delta ===")
    all_ok = True

    cur_pos = np.array([0.5, 0.1, 0.3])
    cur_rpy = np.array([0.1, 0.0, 0.2])
    delta_p = np.array([0.01, 0.02, -0.01])
    delta_r = np.array([0.05, 0.0, -0.05])

    new_pos, new_rpy = apply_ee_delta(cur_pos, cur_rpy, delta_p, delta_r, safe=False)
    all_ok &= _check("位置叠加正确",
                     np.allclose(new_pos, cur_pos + delta_p))
    all_ok &= _check("姿态叠加正确",
                     np.allclose(new_rpy, cur_rpy + delta_r))

    # 安全模式：大增量被截断
    big_p = np.array([0.1, 0.0, 0.0])
    new_pos_safe, _ = apply_ee_delta(cur_pos, cur_rpy, big_p, delta_r, safe=True)
    moved = np.linalg.norm(new_pos_safe - cur_pos)
    all_ok &= _check("safe 模式大增量被截断",
                     moved <= 0.05 + 1e-9,
                     f"实际移动 {moved*1000:.1f}mm")
    return all_ok


def test_frame_transform_roundtrip():
    """验证 arm_utils → frame_transform 坐标变换自洽性（双臂）"""
    print("\n=== 坐标变换自洽性（frame_transform 往返）===")
    all_ok = True

    test_cases = [
        ('right', [430.0,  285.0, 324.0,  10.0,  5.0, -170.0]),
        ('left',  [430.0, -285.0, 324.0, -10.0,  5.0,  170.0]),
    ]
    for arm, fk_xyzabc in test_cases:
        pos_m, rpy_rad = fk_to_base(fk_xyzabc, arm)
        xyzabc_back = base_to_fk(pos_m, rpy_rad, arm)

        pos_err_mm = np.linalg.norm(
            np.array(fk_xyzabc[:3]) - np.array(xyzabc_back[:3])
        )
        rpy_err_deg = max(
            abs(fk_xyzabc[3+i] - xyzabc_back[3+i]) for i in range(3)
        )

        all_ok &= _check(f"{arm} 位置往返误差 < 0.001mm",
                         pos_err_mm < 0.001,
                         f"{pos_err_mm:.4f}mm")
        all_ok &= _check(f"{arm} 姿态往返误差 < 0.001°",
                         rpy_err_deg < 0.001,
                         f"{rpy_err_deg:.4f}°")

        # 打印变换结果供人工核查
        print(f"  [{arm}] FK(臂系): xyz={[f'{v:.1f}' for v in fk_xyzabc[:3]]}mm  "
              f"ABC={[f'{v:.1f}' for v in fk_xyzabc[3:]]}°")
        print(f"  [{arm}] base_link: pos={[f'{v:.4f}' for v in pos_m]}m  "
              f"rpy={[f'{math.degrees(v):.1f}' for v in rpy_rad]}°")

    return all_ok


# ═══════════════════════════════════════════════════════════════════════════════
# 硬件集成测试（需在 Jetson 上运行）
# ═══════════════════════════════════════════════════════════════════════════════

def test_dual_arm_hw():
    """
    DualArm 硬件集成测试：连接 → 读状态 → 验证变换 → 断开
    【不下发任何运动指令，仅读取状态】
    """
    print("\n=== DualArm 硬件集成测试（只读，不运动）===")
    from arm_utils import DualArm

    da = DualArm()
    print("  连接机械臂 ...")
    if not da.connect():
        print(f"  [{FAIL}] 连接失败")
        return False

    print("  读取关节角 ...")
    joints = da.read_joints()
    for arm_str in ('left', 'right'):
        j = joints[arm_str]
        ok = len(j) == 7
        in_lim = check_joints_in_limits(j)
        print(f"  [{PASS if ok else FAIL}] {arm_str}: "
              f"[{', '.join(f'{v:.1f}' for v in j)}]°  "
              f"{'在限位内' if in_lim else '⚠ 超出软限位'}")

    print("  读取 EE 状态 (base_link 系) ...")
    states = da.get_ee_states_base()
    for arm_str in ('left', 'right'):
        pos_m, rpy_rad = states[arm_str]
        if pos_m is None:
            print(f"  [{FAIL}] {arm_str}: FK 失败")
        else:
            print(f"  [{PASS}] {arm_str}: "
                  f"pos=[{pos_m[0]:.4f}, {pos_m[1]:.4f}, {pos_m[2]:.4f}]m  "
                  f"rpy=[{math.degrees(rpy_rad[0]):.1f}, "
                  f"{math.degrees(rpy_rad[1]):.1f}, "
                  f"{math.degrees(rpy_rad[2]):.1f}]°")

    print("  坐标变换验证 ...")
    da.verify_transforms()

    da.release()
    print(f"  [{PASS}] 已断开")
    return True


# ═══════════════════════════════════════════════════════════════════════════════
# 入口
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="arm_utils 测试")
    parser.add_argument("--hw", action="store_true",
                        help="运行硬件集成测试（需连接机械臂）")
    args = parser.parse_args()

    results = []
    print("=" * 60)
    print("  arm_utils 工具函数测试（无硬件依赖）")
    print("=" * 60)

    results.append(test_joint_limits())
    results.append(test_clamp_delta_pos())
    results.append(test_clamp_delta_rpy())
    results.append(test_apply_ee_delta())
    results.append(test_frame_transform_roundtrip())

    n_pass = sum(results)
    n_total = len(results)
    print(f"\n{'='*60}")
    print(f"  {PASS if n_pass == n_total else FAIL}  {n_pass}/{n_total} 组测试通过")
    print(f"{'='*60}")

    if args.hw:
        print("\n" + "=" * 60)
        print("  DualArm 硬件集成测试（需连接机械臂）")
        print("=" * 60)
        test_dual_arm_hw()


if __name__ == "__main__":
    main()
