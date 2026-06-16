#!/usr/bin/env python3
"""
demo.py — 硬件功能验证（无 GR00T 模型）

步骤：
  0. 双臂运动到全零关节角 [0,0,0,0,0,0,0]
  1. 左臂末端 → base_link 原点 (0,0,0) m，末端竖直向下
  2. 左臂夹爪开合 5 次
  3. 左臂末端 → (-0.5,-0.2, 0.5) m
  4. 右臂末端 → base_link 原点 (0,0,0) m，末端竖直向下
  5. 右臂夹爪开合 5 次
  6. 右臂末端 → ( 0.5, 0.2, 0.5) m

每步完成后暂停 5s 并打印双臂末端状态。
摄像头帧率在后台线程中每 2s 打印一次。

用法：
    python3 demo.py              # 真实硬件
    python3 demo.py --mock       # 全部 mock（无机械臂/夹爪连接，逻辑测试用）
"""

import sys
import os
import time
import math
import logging
import threading
import argparse

import numpy as np

# 将 app/ 加入 path（从其他目录运行时）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config_dual as config
from arm_utils import DualArm, MockDualArm
from gripper   import XenseGripper, MockGripper
try:
    from camera import DualCamera
    _CAMERA_AVAILABLE = True
except ImportError:
    _CAMERA_AVAILABLE = False

logging.basicConfig(
    format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
    datefmt='%H:%M:%S',
    level=logging.INFO,
)
logger = logging.getLogger('demo')


# ── 参数 ──────────────────────────────────────────────────────────────────────

STEP_PAUSE      = 5.0    # 每步完成后暂停时间（秒）
ARM_TIMEOUT     = 30.0   # 机械臂到位等待超时（秒）
GRIPPER_TIMEOUT = 10.0   # 夹爪到位等待超时（秒）
OPEN_CLOSE_N    = 5      # 夹爪开合次数

# "末端竖直向下"姿态（base_link 系 roll/pitch/yaw，弧度）
# roll=π 使末端 Z 轴朝向 -Z（即朝下）；pitch/yaw=0
# TODO: 首次上机后确认此值对实际末端朝向正确
EE_DOWN_RPY = (math.pi, 0.0, 0.0)

# 目标位置（base_link 系，米）
# 工作空间约束：base_link_Y < -0.372m（两臂 J0 均在 Y=-0.372m）
# 右臂 J0: Z=+0.035m；左臂 J0: Z=-0.039m（低 74mm），其余相同
#
# 臂坐标系换算：
#   arm_X = (base_Z − J0_Z) × 1000    arm_Y = −(base_X − 0.617) × 1000
#   arm_Z = −(base_Y + 0.372) × 1000  （须 > 0 才可达）
#
# RIGHT_START 与 demo_right.py 的 POS_B 相同，已实测可达。
POS_RIGHT_START = [0.45, -0.55, 0.50]  # 右臂起始，arm≈[465,167,178]mm dist≈525mm ✓
POS_RIGHT_END   = [0.45, -0.65, 0.45]  # 右臂终止，arm≈[415,167,278]mm dist≈527mm
POS_LEFT_START  = [0.45, -0.55, 0.43]  # 左臂起始，arm≈[469,167,178]mm dist≈529mm
POS_LEFT_END    = [0.45, -0.65, 0.38]  # 左臂终止，arm≈[419,167,278]mm dist≈530mm

# 全零关节角（步骤 0）
JOINTS_ZERO = [0.0] * 7


# ═══════════════════════════════════════════════════════════════════════════════
# 摄像头帧率监控（后台线程）
# ═══════════════════════════════════════════════════════════════════════════════

class CameraFpsMonitor:
    """每 2s 打印一次头部/腕部摄像头帧率。"""

    def __init__(self, cam, interval: float = 2.0):
        self._cam      = cam
        self._interval = interval
        self._stop     = False
        self._thread   = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True

    def _run(self):
        counts = {'head': 0, 'wrist': 0}
        t0 = time.time()
        while not self._stop:
            head, wrist = self._cam.read()
            if head  is not None: counts['head']  += 1
            if wrist is not None: counts['wrist'] += 1
            elapsed = time.time() - t0
            if elapsed >= self._interval:
                fps_h = counts['head']  / elapsed
                fps_w = counts['wrist'] / elapsed
                logger.info(f"[camera] head={fps_h:.1f} fps  wrist={fps_w:.1f} fps")
                counts = {'head': 0, 'wrist': 0}
                t0 = time.time()
            time.sleep(0.002)


# ═══════════════════════════════════════════════════════════════════════════════
# 辅助函数
# ═══════════════════════════════════════════════════════════════════════════════

def banner(n: int, desc: str):
    print()
    print('=' * 62)
    print(f'  步骤 {n}：{desc}')
    print('=' * 62)


def print_arm_states(arm: DualArm):
    """读取并打印双臂当前末端位姿（base_link 系）。"""
    states  = arm.get_ee_states_base()
    joints  = arm.read_joints()
    for side in ('left', 'right'):
        pos, rpy = states[side]
        j = joints[side]
        if pos is not None:
            print(f"  [{side}] "
                  f"pos=[{pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}]m  "
                  f"rpy=[{math.degrees(rpy[0]):+.1f},"
                  f"{math.degrees(rpy[1]):+.1f},"
                  f"{math.degrees(rpy[2]):+.1f}]°")
        else:
            print(f"  [{side}] EE 状态读取失败")
        print(f"  [{side}] joints=[{', '.join(f'{v:.1f}' for v in j)}]°")


def pause(label: str, arm: DualArm):
    """打印状态后等待 STEP_PAUSE 秒。"""
    print(f'\n  ── {label} 完成 ──')
    print_arm_states(arm)
    print(f'  暂停 {STEP_PAUSE:.0f}s ...')
    time.sleep(STEP_PAUSE)


def open_close(gripper, side: str, n: int, timeout: float):
    """执行 n 次张开/闭合，打印每次结果。"""
    for i in range(n):
        print(f'  [{side}] 第{i+1}/{n}次 — 张开 ...', end='', flush=True)
        ok = gripper.open(timeout=timeout)
        pos = gripper.get_position()
        print(f' {pos:.1f}mm {"✓" if ok else "✗"}  闭合 ...', end='', flush=True)
        ok = gripper.close(timeout=timeout)
        pos = gripper.get_position()
        print(f' {pos:.1f}mm {"✓" if ok else "✗"}')


# ═══════════════════════════════════════════════════════════════════════════════
# 主程序
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='硬件功能验证 Demo')
    parser.add_argument('--mock', action='store_true',
                        help='全部使用 Mock（无需真实硬件连接）')
    args = parser.parse_args()

    use_mock = args.mock or config.GRIPPER_MOCK

    print()
    print('=' * 62)
    print('  GR00T 硬件功能验证 Demo（无模型）')
    print(f'  模式: {"Mock（无硬件）" if use_mock else "真实硬件"}')
    print('=' * 62)

    # ── 初始化 ────────────────────────────────────────────────────────────────

    logger.info('[初始化] 连接机械臂 ...')
    if use_mock:
        arm = MockDualArm()
        arm.connect()   # 无操作，始终成功
    else:
        arm = DualArm()
        if not arm.connect():
            logger.error('机械臂连接失败，退出')
            return

    logger.info('[初始化] 连接夹爪 ...')
    if use_mock:
        gripper_l = MockGripper(name='left')
        gripper_r = MockGripper(name='right')
    else:
        gripper_l = XenseGripper(config.GRIPPER_MAC_LEFT,
                                  name='left',
                                  vmax=config.GRIPPER_VMAX,
                                  fmax=config.GRIPPER_FMAX,
                                  tol =config.GRIPPER_TOL)
        gripper_r = XenseGripper(config.GRIPPER_MAC_RIGHT,
                                  name='right',
                                  vmax=config.GRIPPER_VMAX,
                                  fmax=config.GRIPPER_FMAX,
                                  tol =config.GRIPPER_TOL)
        if not gripper_l.connect():
            logger.error('左臂夹爪连接失败，退出')
            arm.release(); return
        if not gripper_r.connect():
            logger.error('右臂夹爪连接失败，退出')
            gripper_l.disconnect(); arm.release(); return

    logger.info('[初始化] 启动摄像头 ...')
    cam_ok = False
    cam = None
    fps_monitor = None
    if not _CAMERA_AVAILABLE:
        logger.warning('pyrealsense2 未安装，跳过摄像头（帧率监控不可用）')
    else:
        cam = DualCamera()
        try:
            cam.start()
            cam_ok = True
            fps_monitor = CameraFpsMonitor(cam)
            fps_monitor.start()
        except Exception as e:
            logger.warning(f'摄像头启动失败（跳过）: {e}')

    logger.info('[初始化完成]')

    try:
        # ── 步骤 0：双臂全零关节角 ────────────────────────────────────────────
        banner(0, '双臂运动到全零关节角 [0°×7]')
        ok_l, ok_r = arm.move_joints_both_sync(JOINTS_ZERO, JOINTS_ZERO, ARM_TIMEOUT)
        print(f'  左臂: {"✓ 到位" if ok_l else "✗ 超时/失败"}')
        print(f'  右臂: {"✓ 到位" if ok_r else "✗ 超时/失败"}')
        pause('步骤 0', arm)

        # ── 步骤 1：左臂 → 起始位置 ──────────────────────────────────────────
        banner(1, f'左臂末端 → {POS_LEFT_START} m，末端竖直向下')
        ok = arm.move_to_ee_base_sync('left', POS_LEFT_START, EE_DOWN_RPY, ARM_TIMEOUT)
        print(f'  左臂: {"✓ 到位" if ok else "✗ IK 无解或超时"}')
        pause('步骤 1', arm)

        # ── 步骤 2：左臂夹爪开合 5 次 ────────────────────────────────────────
        banner(2, f'左臂夹爪开合 {OPEN_CLOSE_N} 次')
        open_close(gripper_l, 'left', OPEN_CLOSE_N, GRIPPER_TIMEOUT)
        pause('步骤 2', arm)

        # ── 步骤 3：左臂 → (-0.5,-0.2,0.5) ──────────────────────────────────
        banner(3, f'左臂末端 → {POS_LEFT_END} m')
        ok = arm.move_to_ee_base_sync('left', POS_LEFT_END, EE_DOWN_RPY, ARM_TIMEOUT)
        print(f'  左臂: {"✓ 到位" if ok else "✗ IK 无解或超时"}')
        pause('步骤 3', arm)

        # ── 步骤 4：右臂 → 起始位置 ──────────────────────────────────────────
        banner(4, f'右臂末端 → {POS_RIGHT_START} m，末端竖直向下')
        ok = arm.move_to_ee_base_sync('right', POS_RIGHT_START, EE_DOWN_RPY, ARM_TIMEOUT)
        print(f'  右臂: {"✓ 到位" if ok else "✗ IK 无解或超时"}')
        pause('步骤 4', arm)

        # ── 步骤 5：右臂夹爪开合 5 次 ────────────────────────────────────────
        banner(5, f'右臂夹爪开合 {OPEN_CLOSE_N} 次')
        open_close(gripper_r, 'right', OPEN_CLOSE_N, GRIPPER_TIMEOUT)
        pause('步骤 5', arm)

        # ── 步骤 6：右臂 → (0.5,0.2,0.5) ────────────────────────────────────
        banner(6, f'右臂末端 → {POS_RIGHT_END} m')
        ok = arm.move_to_ee_base_sync('right', POS_RIGHT_END, EE_DOWN_RPY, ARM_TIMEOUT)
        print(f'  右臂: {"✓ 到位" if ok else "✗ IK 无解或超时"}')
        pause('步骤 6', arm)

        print()
        print('=' * 62)
        print('  全部步骤完成 ✓')
        print('=' * 62)

    except KeyboardInterrupt:
        print('\nCtrl+C，正在安全退出 ...')

    finally:
        if fps_monitor:
            fps_monitor.stop()
        if cam_ok:
            cam.close()
        if not use_mock:
            gripper_l.disconnect()
            gripper_r.disconnect()
        logger.info('回准备位 ...')
        arm.go_home()
        arm.release()
        logger.info('已安全退出')


if __name__ == '__main__':
    main()
