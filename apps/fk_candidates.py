#!/usr/bin/env python3
import sys, os, math
sys.path.insert(0, os.path.expanduser('~/work/app'))
sys.path.insert(0, os.path.join(
    os.path.expanduser('~/work'),
    'TJ_marvin/TJ_FX_ROBOT_CONTRL_SDK-master'))

import config as cfg
from SDK_PYTHON.fx_kine import Marvin_Kine
from frame_transform import fk_to_base

kk0 = Marvin_Kine(); kk0.log_switch(0)
ini = kk0.load_config(arm_type=0, config_path=cfg.CFG_FILE)
assert ini, "config load failed"

def fk_arm(arm_idx, joints_deg):
    kk = Marvin_Kine(); kk.log_switch(0)
    kk.initial_kine(robot_type=ini['TYPE'][arm_idx],
                    dh=ini['DH'][arm_idx],
                    pnva=ini['PNVA'][arm_idx],
                    j67=ini['BD'][arm_idx])
    xyzabc = kk.fkine(joints_deg)
    arm_str = 'left' if arm_idx == 0 else 'right'
    pos, rpy = fk_to_base(xyzabc, arm_str)
    dist = math.sqrt(sum(v**2 for v in xyzabc[:3]))
    return xyzabc, pos, rpy, dist

def show(label, arm_idx, joints):
    xyzabc, pos, rpy, dist = fk_arm(arm_idx, joints)
    arm_str = 'left' if arm_idx == 0 else 'right'
    print(f"\n  [{arm_str}] {label}")
    print(f"    joints:     [{', '.join(f'{v:+.1f}' for v in joints)}]°")
    print(f"    arm-frame:  [{xyzabc[0]:.1f}, {xyzabc[1]:.1f}, {xyzabc[2]:.1f}]mm  "
          f"ABC=[{xyzabc[3]:.1f},{xyzabc[4]:.1f},{xyzabc[5]:.1f}]°  J0距={dist:.0f}mm")
    print(f"    base_link:  [{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}]m  "
          f"rpy=[{math.degrees(rpy[0]):+.1f},{math.degrees(rpy[1]):+.1f},{math.degrees(rpy[2]):+.1f}]°")
    return xyzabc, pos

LEFT_HOME = [89.0, -61.0, -88.0, -86.0, 61.0, -0.2, 2.6]

print("=" * 72)
print("  FK 候选分析：寻找使右臂 base_link EE 与左臂仅 Z 不同的关节角")
print("=" * 72)

_, left_pos = show("HOME（基准）", 0, LEFT_HOME)
print(f"\n  目标：右臂 base_link X={left_pos[0]:+.4f} Y={left_pos[1]:+.4f}，Z 可不同")

candidates = [
    ("原始（与左臂相同）",       [ 89,-61,-88,-86, 61,-0.2,  2.6]),
    ("J1 取反",                 [-89,-61,-88,-86, 61,-0.2,  2.6]),
    ("奇数轴取反 J1/J3/J5/J7", [-89,-61, 88,-86,-61,-0.2, -2.6]),
    ("J1+J3 取反",              [-89,-61, 88,-86, 61,-0.2,  2.6]),
    ("J1+J5 取反",              [-89,-61,-88,-86,-61,-0.2,  2.6]),
    ("全部取反",                [-89, 61, 88, 86,-61, 0.2, -2.6]),
    ("J3+J5+J7 取反",           [ 89,-61, 88,-86,-61,-0.2, -2.6]),
]

print("\n" + "=" * 72)
for label, j in candidates:
    _, rpos = show(label, 1, j)
    dx = abs(rpos[0] - left_pos[0])
    dy = abs(rpos[1] - left_pos[1])
    ok = "✓ X,Y匹配！" if dx < 0.005 and dy < 0.005 else f"✗ ΔX={dx*1000:.1f}mm ΔY={dy*1000:.1f}mm"
    print(f"    --> {ok}")

