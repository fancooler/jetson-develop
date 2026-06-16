"""试探 SDK ↔ training 关节空间的符号映射，目视调 HOME 姿态。

用法（在 Jetson 上）：
  cd ~/work/app
  python3 test/try_mapped_home.py R_mask L_mask

参数：
  R_mask / L_mask: 7 个字符的符号串，每个字符 '+' 或 '-'
                   '+' = 用 metadata mean 原值
                   '-' = 对该关节取反

举例：
  # baseline（当前夹爪朝上的姿态）
  python3 test/try_mapped_home.py "+++++++" "+++++++"

  # 推测 1：按 URDF 负向 axis 反（右臂 J3/J5，左臂 J1/J2；J4 不动以避软限位）
  python3 test/try_mapped_home.py "++-+-++" "--+++++"

  # 推测 2：手腕（J5/J6/J7）反，看末端朝向变化
  python3 test/try_mapped_home.py "++++---" "++++---"

每个映射会先做软限位检查，超限直接拒绝；通过则慢速 go_home 到目标位置。
完成后打印实际 obs，便于对照构型图分析每个关节的物理含义。
"""
import os
import sys
import logging

# 工具脚本在 test/ 下，需要把 app/ 加到路径
APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP_DIR)

logging.basicConfig(format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
                    datefmt='%H:%M:%S', level=logging.INFO)

from arm_utils import DualArm, check_joints_in_limits, _go_home_arms, _cfg
import config_dual as cfg

# metadata action.{right,left}_arm.mean（弧度转度，整数化）
MEAN_R = [ 42.0,   4.0,  15.0,  -79.0,  31.0,  14.0,  10.0]
MEAN_L = [-52.0, -62.0, -99.0,  -90.0,  18.0,  -8.0, -35.0]


def parse_mask(s, label):
    s = s.replace('‑', '-').strip()
    if len(s) != 7 or any(c not in '+-' for c in s):
        raise ValueError(f"{label} mask 必须是 7 个 +/- 字符，收到 '{s}'")
    return [1.0 if c == '+' else -1.0 for c in s]


def apply_mask(joints, mask):
    return [j * m for j, m in zip(joints, mask)]


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    mask_R = parse_mask(sys.argv[1], 'R')
    mask_L = parse_mask(sys.argv[2], 'L')

    target_R = apply_mask(MEAN_R, mask_R)
    target_L = apply_mask(MEAN_L, mask_L)

    print(f"\nR mask: {sys.argv[1]}  → target R = {[round(x,1) for x in target_R]}°")
    print(f"L mask: {sys.argv[2]}  → target L = {[round(x,1) for x in target_L]}°\n")

    if not check_joints_in_limits(target_R):
        print("[FAIL] R 超软限位（拒绝执行）")
        sys.exit(2)
    if not check_joints_in_limits(target_L):
        print("[FAIL] L 超软限位（拒绝执行）")
        sys.exit(2)

    da = DualArm()
    if not da.connect():
        print("[FAIL] connect failed")
        sys.exit(1)

    s = da.read_all_states()
    print(f"[obs] before: R = {[round(x,1) for x in s['joints']['right']]}")
    print(f"[obs] before: L = {[round(x,1) for x in s['joints']['left']]}\n")

    override = {'right': target_R, 'left': target_L}
    print("[move] go to mapped HOME ...")
    ok = _go_home_arms(da._robot, da._dcss, ['left', 'right'],
                       _cfg(), home_override=override)
    print(f"[result] {ok}\n")

    s = da.read_all_states()
    print(f"[obs] after : R = {[round(x,1) for x in s['joints']['right']]}")
    print(f"[obs] after : L = {[round(x,1) for x in s['joints']['left']]}")

    da.release()


if __name__ == '__main__':
    main()
