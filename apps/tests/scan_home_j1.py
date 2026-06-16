#!/usr/bin/env python3
"""scan_home_j1.py — 扫描右臂不同 J1 home 值对任务 IK 解的影响

用法（在 Jetson 上运行）：
  python3 test/scan_home_j1.py

输出：
  1. 不同 J1 home 值下 FK 得到的 EE 位置（base_link 系）
  2. 以当前 home EE 的 X/Y 为目标，目标 Z 设为取件高度，
     从该 J1 home 出发求 IK，显示 J2 结果
  3. 标记 J2 是否超过软限位（-90°）

目的：找到 J1 home 值使得整个任务 IK 链条中 J2 保持在安全范围内。
"""

import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'TJ_marvin/TJ_FX_ROBOT_CONTRL_SDK-master'))

import config_dual as cfg
from SDK_PYTHON.fx_kine import Marvin_Kine, FX_InvKineSolvePara as FX_IKPara
from frame_transform import fk_to_base, base_to_fk

# ── 初始化运动学 ──────────────────────────────────────────────────────────────
kk0 = Marvin_Kine(); kk0.log_switch(0)
ini = kk0.load_config(arm_type=0, config_path=cfg.CFG_FILE)
assert ini, "config load failed"

ARM_IDX = 1  # right arm

def make_kine():
    kk = Marvin_Kine(); kk.log_switch(0)
    kk.initial_kine(
        robot_type=ini['TYPE'][ARM_IDX],
        dh=ini['DH'][ARM_IDX],
        pnva=ini['PNVA'][ARM_IDX],
        j67=ini['BD'][ARM_IDX],
    )
    return kk

def fk(joints):
    """返回 (xyzabc_mm_deg, pos_m, rpy_rad) 或 None"""
    kk = make_kine()
    fk_mat = kk.fk(joints)
    if not fk_mat:
        return None
    xyzabc = kk.mat4x4_to_xyzabc(fk_mat)
    if not xyzabc:
        return None
    pos_m, rpy_rad = fk_to_base(xyzabc, 'right')
    return xyzabc, pos_m, rpy_rad

def solve_ik(target_xyzabc, ref_joints):
    """从 ref_joints 出发，对 target_xyzabc（臂系 mm/deg）求 IK。
    返回 joints list 或 None。"""
    kk = make_kine()
    tcp_mat = kk.xyzabc_to_mat4x4(target_xyzabc)
    if not tcp_mat:
        return None
    tcp_flat = [tcp_mat[r][c] for r in range(4) for c in range(4)]

    sp = FX_IKPara()
    sp.set_input_ik_target_tcp(tcp_flat)
    ref = list(ref_joints)
    if abs(ref[3]) < 0.5:
        ref[3] = 1.0
    sp.set_input_ik_ref_joint(ref)
    sp.set_input_ik_zsp_type(0)

    res = kk.ik(sp)
    if not res or res.m_Output_IsOutRange or res.m_Output_IsJntExd:
        return None
    return res.m_Output_RetJoint.to_list()

# ── 当前 home 的 FK（作为任务目标 XY 参考）──────────────────────────────────
current_home = cfg.HOME_JOINTS_RIGHT
result = fk(current_home)
assert result, "当前 home FK 失败"
home_xyzabc, home_pos, home_rpy = result

print("=" * 70)
print("  当前 home FK（右臂）")
print(f"  joints: {[f'{v:+.1f}' for v in current_home]}")
print(f"  base_link: X={home_pos[0]:+.4f}  Y={home_pos[1]:+.4f}  Z={home_pos[2]:+.4f} m")
print(f"  arm-frame: X={home_xyzabc[0]:.1f}  Y={home_xyzabc[1]:.1f}  Z={home_xyzabc[2]:.1f} mm")

# ── 任务目标：optical module 取件位（base_link 坐标系）──────────────────────
# optical module Isaac Lab world: [0.32, -0.10, 0.84] m
# 变换：base_X = -world_Y, base_Y = BASE_OFFSET - world_Z, base_Z = world_X
# → base_link: [X=0.10, Y≈0, Z=0.32]
# 取件时 EE 在 module 正上方，Z 稍低（训练数据最低 0.216m，这里用 0.25m 留余量）
PICK_POS_M = [0.10, 0.0, 0.28]  # base_link [X, Y, Z]，可根据实测微调
PICK_RPY   = list(home_rpy)  # 姿态与 home 相同

pick_xyzabc = base_to_fk(PICK_POS_M, PICK_RPY, 'right')

print(f"\n  取件目标（base_link）: X={PICK_POS_M[0]:+.4f}  Y={PICK_POS_M[1]:+.4f}  Z={PICK_POS_M[2]:+.4f} m")
print(f"  取件目标（arm-frame）: {[f'{v:.1f}' for v in pick_xyzabc]}")

J2_LIMIT = -90.0   # 当前软限位（度）

# ── 扫描 J1 home 值 ───────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print(f"  J1 home 扫描（其余关节保持 {current_home[1:]} 不变）")
print(f"  {'J1_home':>8}  {'home_Z(m)':>10}  {'home_Y(m)':>10}  "
      f"{'IK_J2(°)':>10}  {'IK_J1(°)':>10}  状态")
print("-" * 70)

for j1 in range(-45, -95, -5):
    candidate = [float(j1)] + list(current_home[1:])
    r = fk(candidate)
    if r is None:
        print(f"  {j1:>8.1f}  FK失败")
        continue
    _, cpos, crpy = r

    # 用这个 home 作为 IK 起点，解取件位
    ik_joints = solve_ik(pick_xyzabc, candidate)
    if ik_joints is None:
        status = "IK无解"
        print(f"  {j1:>8.1f}  {cpos[2]:>10.4f}  {cpos[1]:>10.4f}  "
              f"{'—':>10}  {'—':>10}  {status}")
    else:
        j2_result = ik_joints[1]
        j1_result = ik_joints[0]
        safe = j2_result >= J2_LIMIT
        status = "OK" if safe else f"!! J2超限({j2_result:.1f}<{J2_LIMIT})"
        print(f"  {j1:>8.1f}  {cpos[2]:>10.4f}  {cpos[1]:>10.4f}  "
              f"  {j2_result:>8.1f}  {j1_result:>8.1f}  {status}")

print("=" * 70)
print("  说明：IK 从 J1_home 出发只求一步（home→取件），")
print("  实际任务是多步连续解，结果供参考，选 J2 余量最大的 J1_home 上机验证。")
