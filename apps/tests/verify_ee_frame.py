#!/usr/bin/env python3
"""verify_ee_frame.py — 判定 SDK-FK + frame_transform 是否复现 Isaac/URDF 的 base_link EE

背景：
  模型在 Isaac Sim 训练，state.ee_pose 是 Isaac 按 URDF 运动学算出来的。
  真机推理时 ee_pose 走的是另一条链：
      天机 SDK FK(关节角) → frame_transform.fk_to_base → base_link → 世界系
  若这条链与 URDF 不一致，state.ee_pose 就喂了错的坐标给模型 → sim 行真机不行。

  本脚本对同一组关节角：
    (a) 跑 SDK Marvin_Kine.fkine + frame_transform.fk_to_base → base_link 位姿
    (b) 对照下面 URDF_REF（用 ros_model_260418_1 URDF 链独立算的 base_link flange 位姿）
  逐轴打印差值。纯运动学，不需连机器人。

用法（Jetson）：
    cd ~/work/app
    python3 test/verify_ee_frame.py

判读：
  - 位置差 < ~5mm 且姿态差 < ~2° → SDK 关节约定==URDF 且 frame_transform 正确，
    ee_pose 链没问题，坐标错误不在这里（去查关节符号/动作执行路径）。
  - 某轴差很大 → 锁定 frame_transform._R_RIGHT/_R_LEFT 或 SDK 关节约定的错位。
"""
import os
import sys
import math
import numpy as np

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP_DIR)
# 与 fk_candidates.py / scan_home_j1.py 一致：SDK 在 ~/work/TJ_marvin/...
sys.path.insert(0, os.path.join(
    os.path.expanduser('~/work'),
    'TJ_marvin/TJ_FX_ROBOT_CONTRL_SDK-master'))

import config as cfg
from SDK_PYTHON.fx_kine import Marvin_Kine
from frame_transform import fk_to_base

np.set_printoptions(precision=4, suppress=True)

# ── URDF (ros_model_260418_1) 独立算出的 base_link flange 参考 ──
# 字段: pos_m[3], rpy_deg[3]（base_link 系）
URDF_REF = {
    "HOME": {
        "R": ([0.33915, -0.52028, 0.51858], [-122.200, -0.556, -168.714]),
        "L": ([0.54659, 0.01223, -0.29848], [167.529, 43.721, -41.486]),
        "qr": [42, 4, 15, -79, 31, 14, 10],
        "ql": [-52, -62, -99, -90, 18, -8, -35],
    },
    "ZERO": {
        "R": ([0.61736, -0.37205, 0.81092], [-90.0, -90.0, 180.0]),
        "L": ([0.61736, -0.37205, -0.81408], [-90.0, -90.0, 180.0]),
        "qr": [0, 0, 0, 0, 0, 0, 0],
        "ql": [0, 0, 0, 0, 0, 0, 0],
    },
    "J2_30": {
        "R": ([0.31686, -0.37205, 0.73040], [-90.0, -60.0, -180.0]),
        "L": ([0.61736, -0.37205, -0.81408], [-90.0, -90.0, 180.0]),
        "qr": [0, 30, 0, 0, 0, 0, 0],
        "ql": [0, 0, 0, 0, 0, 0, 0],
    },
}


def main():
    kk0 = Marvin_Kine(); kk0.log_switch(0)
    ini = kk0.load_config(arm_type=0, config_path=cfg.CFG_FILE)
    assert ini, "SDK config 加载失败"

    def sdk_fk_base(arm_idx, joints_deg, arm_str):
        kk = Marvin_Kine(); kk.log_switch(0)
        kk.initial_kine(robot_type=ini['TYPE'][arm_idx],
                        dh=ini['DH'][arm_idx],
                        pnva=ini['PNVA'][arm_idx],
                        j67=ini['BD'][arm_idx])
        fk_mat = kk.fk(joints_deg)           # 4x4
        xyzabc = kk.mat4x4_to_xyzabc(fk_mat) # 臂坐标系 [X_mm,Y_mm,Z_mm,A,B,C]
        pos_m, rpy_rad = fk_to_base(xyzabc, arm_str)   # → base_link
        rpy_deg = [math.degrees(v) for v in rpy_rad]
        return np.array(pos_m), np.array(rpy_deg), xyzabc

    for name, ref in URDF_REF.items():
        print("=" * 78)
        print(f"  配置 {name}")
        print("=" * 78)
        for side, arm_idx, arm_str, qkey in [
            ("R", 1, "right", "qr"),
            ("L", 0, "left",  "ql"),
        ]:
            q = ref[qkey]
            pos_sdk, rpy_sdk, xyzabc = sdk_fk_base(arm_idx, q, arm_str)
            pos_ref = np.array(ref[side][0])
            rpy_ref = np.array(ref[side][1])

            dpos = pos_sdk - pos_ref
            # 姿态差按角度环绕处理
            drpy = ((rpy_sdk - rpy_ref + 180) % 360) - 180

            print(f"  [{arm_str}] joints(deg) = {q}")
            print(f"     SDK 臂坐标系 xyzabc   = {np.array(xyzabc)}")
            print(f"     SDK→base_link  pos(m) = {pos_sdk}   rpy(deg)= {rpy_sdk}")
            print(f"     URDF ref       pos(m) = {pos_ref}   rpy(deg)= {rpy_ref}")
            print(f"     Δpos(mm) = {dpos*1000}   |Δpos|={np.linalg.norm(dpos)*1000:.1f}mm")
            print(f"     Δrpy(deg)= {drpy}")
            verdict = "✓ 一致" if (np.linalg.norm(dpos) < 0.005 and np.max(np.abs(drpy)) < 2) \
                      else "✗ 不一致 —— 坐标错位在此"
            print(f"     → {verdict}")
            print()


if __name__ == "__main__":
    main()
