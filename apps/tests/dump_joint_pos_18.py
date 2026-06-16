#!/usr/bin/env python3
"""dump_joint_pos_18.py — sim/real 标定专用：dump 实物当前 state.joint_pos[18]

用途：
  state.joint_pos[18] 的内部布局（哪个 index 对应哪根关节）没有任何文档说明，
  必须 sim/real 在同一姿态下分别 dump 18 维向量，逐元素对比才能确认。

用法（Jetson 上）：
  1. 先用 set_joints_home.py 把双臂摆到目标姿态：
       python3 test/set_joints_home.py "42,4,15,-79,31,14,10" "-52,-62,-99,-90,18,-8,-35"
  2. 不要松手/不要让机器人动，直接跑：
       python3 test/dump_joint_pos_18.py

  也支持可选参数手动指定夹爪 Xense mm（默认 50.0 = 全开）：
       python3 test/dump_joint_pos_18.py --gripper-r 50 --gripper-l 50

输出：
  - SDK 原始读数（左右臂 7 度，左右夹爪 Xense mm）
  - infer_dual._build_joint_pos_18 拼装后的 18 维 state.joint_pos
  - 一段 REAL_* 报告块，直接复制贴给算法同事，他在 Isaac Sim 端复现同关节角后
    dump 一份 sim 侧 state.joint_pos[18]，两份逐 index 对照
"""
import os
import sys
import argparse
import logging
import numpy as np

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP_DIR)

logging.basicConfig(format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
                    datefmt='%H:%M:%S', level=logging.WARNING)

from arm_utils import DualArm
import infer_dual


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gripper-r', type=float, default=50.0,
                        help='右夹爪 Xense 位置(mm)，默认 50.0=全开')
    parser.add_argument('--gripper-l', type=float, default=50.0,
                        help='左夹爪 Xense 位置(mm)，默认 50.0=全开')
    parser.add_argument('--label', type=str, default='custom',
                        help='姿态标签（如 home / zero / probe1），用于报告头')
    args = parser.parse_args()

    da = DualArm()
    if not da.connect():
        print("[FAIL] connect failed")
        sys.exit(1)

    try:
        states = da.read_all_states()
        joints_r = list(states['joints']['right'])   # 度
        joints_l = list(states['joints']['left'])    # 度
        gr_mm = args.gripper_r
        gl_mm = args.gripper_l

        # 用推理代码的拼装函数得到 18 维 state.joint_pos
        jp18 = infer_dual._build_joint_pos_18(joints_r, joints_l, gr_mm, gl_mm)

        print()
        print("=" * 78)
        print(f"  RAW SDK 读数（姿态标签: {args.label}）")
        print("=" * 78)
        print(f"  右臂关节(度) [J1..J7]: " +
              ", ".join(f"{v:+8.3f}" for v in joints_r))
        print(f"  左臂关节(度) [J1..J7]: " +
              ", ".join(f"{v:+8.3f}" for v in joints_l))
        print(f"  右夹爪 Xense (mm): {gr_mm:.2f}")
        print(f"  左夹爪 Xense (mm): {gl_mm:.2f}")
        print()

        print("=" * 78)
        print("  state.joint_pos[18] (按 infer_dual._build_joint_pos_18 拼装)")
        print("  单位：[0:14] rad（关节角），[14:18] m（夹爪指偏移）")
        print("=" * 78)
        guess_labels = [
            ("[ 0] r_J1", "rad"),
            ("[ 1] r_J2", "rad"),
            ("[ 2] r_J3", "rad"),
            ("[ 3] r_J4", "rad"),
            ("[ 4] r_J5", "rad"),
            ("[ 5] r_J6", "rad"),
            ("[ 6] r_J7", "rad"),
            ("[ 7] l_J1", "rad"),
            ("[ 8] l_J2", "rad"),
            ("[ 9] l_J3", "rad"),
            ("[10] l_J4", "rad"),
            ("[11] l_J5", "rad"),
            ("[12] l_J6", "rad"),
            ("[13] l_J7", "rad"),
            ("[14] r_finger1_offset", "m"),
            ("[15] r_finger2_offset", "m"),
            ("[16] l_finger1_offset", "m"),
            ("[17] l_finger2_offset", "m"),
        ]
        for i, (label, unit) in enumerate(guess_labels):
            v = float(jp18[i])
            extra = f"  ({np.degrees(v):+8.3f}°)" if unit == "rad" else ""
            print(f"  {label:25s} = {v:+11.6f} {unit}{extra}")

        print()
        print("⚠  上面 [0:14] 的 label 是 infer_dual.py 当前假设的布局，待 sim 端 dump 同")
        print("   关节角后逐 index 核对。若 sim 端 index 4 对应右 J5 而 real 端 index 4")
        print("   实际是右 J4，标定就发现了第一个错位点。")
        print()
        print("=" * 78)
        print("  📋 复制下面这一整块给算法同事，请他在 Isaac Sim 里 set 到同样的关节角")
        print("  （HOME_JOINTS_RIGHT/LEFT 即可），然后从 articulation_view 读")
        print("  state.joint_pos[18]，再贴回一份 SIM_JOINT_POS_18 对照")
        print("=" * 78)
        print()
        print(f"# REAL_POSE_LABEL : {args.label}")
        print(f"REAL_JOINTS_R_DEG = {joints_r}")
        print(f"REAL_JOINTS_L_DEG = {joints_l}")
        print(f"REAL_GRIPPER_R_MM = {gr_mm}")
        print(f"REAL_GRIPPER_L_MM = {gl_mm}")
        print(f"REAL_JOINT_POS_18 = {[float(v) for v in jp18]}")
        print()
        print("# Isaac Sim 端等价操作（伪代码，请算法同事按现场 API 实现）：")
        print("#   joints_r_rad = np.deg2rad(REAL_JOINTS_R_DEG)")
        print("#   joints_l_rad = np.deg2rad(REAL_JOINTS_L_DEG)")
        print("#   articulation_view.set_joint_positions(<对应 14 个臂关节>)")
        print("#   sim.step(N)  # 跑稳态")
        print("#   jp18_sim = articulation_view.get_joint_positions()  # shape [18]")
        print("#   print(f'SIM_JOINT_POS_18 = {jp18_sim.tolist()}')")

    finally:
        da.release()


if __name__ == "__main__":
    main()
