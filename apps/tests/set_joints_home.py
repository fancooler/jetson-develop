"""直接指定关节角作为 HOME，慢速移动到位（无 mask、无映射）。

用法：
  cd ~/work/app
  python3 test/set_joints_home.py "j1,j2,j3,j4,j5,j6,j7" "j1,j2,j3,j4,j5,j6,j7"

举例：
  # warmup 5 次推理平均（之前 std<2° 的"模型期望任务起点"）
  python3 test/set_joints_home.py \\
      "39.5,6.8,24.0,-71.3,66.9,9.0,-36.9" \\
      "-11.2,-55.0,-79.2,-104.1,7.8,-22.8,-21.2"

  # metadata action mean（当前 config_dual 默认值）
  python3 test/set_joints_home.py \\
      "42,4,15,-79,31,14,10" \\
      "-52,-62,-99,-90,18,-8,-35"
"""
import os
import sys
import logging

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP_DIR)

logging.basicConfig(format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
                    datefmt='%H:%M:%S', level=logging.INFO)

from arm_utils import DualArm, check_joints_in_limits, _go_home_arms, _cfg


def parse_joints(s, label):
    parts = [p.strip() for p in s.split(',')]
    if len(parts) != 7:
        raise ValueError(f"{label} 需要 7 个用逗号分隔的关节角，收到 {len(parts)} 个")
    return [float(p) for p in parts]


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    target_R = parse_joints(sys.argv[1], 'R')
    target_L = parse_joints(sys.argv[2], 'L')

    print(f"\nR target = {[round(x,1) for x in target_R]}°")
    print(f"L target = {[round(x,1) for x in target_L]}°\n")

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
    print("[move] go to target ...")
    ok = _go_home_arms(da._robot, da._dcss, ['left', 'right'],
                       _cfg(), home_override=override)
    print(f"[result] {ok}\n")

    s = da.read_all_states()
    print(f"[obs] after : R = {[round(x,1) for x in s['joints']['right']]}")
    print(f"[obs] after : L = {[round(x,1) for x in s['joints']['left']]}")

    da.release()


if __name__ == '__main__':
    main()
