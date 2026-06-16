#!/usr/bin/env python3
"""
test_gripper.py — Xense 夹爪交互测试

源自 xense/gripper_test.py，调整为从 app/test/ 目录运行，
直接连 xensegripper SDK（不经过 app/gripper.py 封装），
方便逐项验证夹爪硬件功能。

用法：
    cd app/test
    python3 test_gripper.py               # 默认左臂夹爪
    python3 test_gripper.py --arm right   # 右臂夹爪
    python3 test_gripper.py --mac 3ad820773a85
    python3 test_gripper.py --test o      # 非交互：直接跑开合测试
"""

import sys
import os
import time
import argparse
import threading
import logging

# 将 app/ 加入 path，便于 import config
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config_dual as config

logging.basicConfig(
    format='%(asctime)s %(levelname)-5s %(message)s',
    datefmt='%H:%M:%S',
    level=logging.INFO,
)
logger = logging.getLogger("test_gripper")

# ── 配置（默认值从 config 读取）──────────────────────────────────────────────
MAC_LEFT     = config.GRIPPER_MAC_LEFT
MAC_RIGHT    = config.GRIPPER_MAC_RIGHT
DEFAULT_ARM  = "left"
DEFAULT_VMAX = config.GRIPPER_VMAX
DEFAULT_FMAX = config.GRIPPER_FMAX
DEFAULT_TOL  = config.GRIPPER_TOL
POS_OPEN     = config.GRIPPER_POS_OPEN
POS_CLOSE    = config.GRIPPER_POS_CLOSE
STATUS_HZ    = 10   # 状态打印频率（Hz）


# ── 连接 ─────────────────────────────────────────────────────────────────────

def connect(mac=None, arm=None, port=None):
    try:
        import os as _os
        _os.environ.setdefault('QT_API', 'pyside6')
        from xensegripper import XenseGripper
    except ImportError:
        logger.error("xensegripper 未安装。请运行：pip install xensegripper")
        sys.exit(1)

    if port:
        logger.info(f"串口连接: {port}")
        gripper = XenseGripper.create(port=port)
    else:
        if not mac:
            mac = MAC_RIGHT if arm == "right" else MAC_LEFT
        logger.info(f"TCP 连接: arm={arm or 'left'}  mac={mac}")
        gripper = XenseGripper.create(mac_addr=mac)

    logger.info("连接成功")
    return gripper


# ── 状态显示 ─────────────────────────────────────────────────────────────────

def print_status(gripper, once=False):
    """打印一次状态，或后台循环打印。"""
    def _fmt(status):
        if not isinstance(status, dict):
            return f"  状态读取失败: {status}"
        pos  = status.get('position',    '?')
        vel  = status.get('velocity',    '?')
        frc  = status.get('force',       '?')
        temp = status.get('temperature', '?')
        bar  = int(float(pos) / 85 * 30) if isinstance(pos, (int, float)) else 0
        return (f"  位置: {pos:6.2f} mm  [{('#'*bar).ljust(30)}]  "
                f"速度: {vel:+6.2f} mm/s  力: {frc:+6.2f} N  温度: {temp:.1f}°C")

    if once:
        status = gripper.get_gripper_status()
        print(_fmt(status))
        return

    stop_event = threading.Event()

    def _loop():
        while not stop_event.is_set():
            try:
                status = gripper.get_gripper_status()
                print(f"\r{_fmt(status)}", end="", flush=True)
            except Exception as e:
                print(f"\r  状态读取异常: {e}", end="", flush=True)
            time.sleep(1.0 / STATUS_HZ)
        print()  # 换行

    t = threading.Thread(target=_loop, daemon=True)
    t.start()
    return stop_event


# ── 各测试项 ─────────────────────────────────────────────────────────────────

def test_status(gripper):
    print("\n[状态读取]")
    print_status(gripper, once=True)


def test_open_close(gripper):
    print("\n[开合测试]")
    print(f"  → 张开 ({POS_OPEN:.0f} mm)...")
    ok = gripper.set_position_sync(POS_OPEN,  vmax=DEFAULT_VMAX,
                                   fmax=DEFAULT_FMAX, tolerance=DEFAULT_TOL, timeout=8.0)
    print(f"    {'到位 ✓' if ok else '超时 ✗'}")
    print_status(gripper, once=True)
    time.sleep(0.5)

    print(f"  → 闭合 ({POS_CLOSE:.0f} mm)...")
    ok = gripper.set_position_sync(POS_CLOSE, vmax=DEFAULT_VMAX,
                                   fmax=DEFAULT_FMAX, tolerance=DEFAULT_TOL, timeout=8.0)
    print(f"    {'到位 ✓' if ok else '超时 ✗'}")
    print_status(gripper, once=True)
    time.sleep(0.5)

    print("  → 回到中间 (42.5 mm)...")
    ok = gripper.set_position_sync(42.5, vmax=DEFAULT_VMAX,
                                   fmax=DEFAULT_FMAX, tolerance=DEFAULT_TOL, timeout=8.0)
    print(f"    {'到位 ✓' if ok else '超时 ✗'}")
    print_status(gripper, once=True)


def test_position_sweep(gripper):
    print("\n[位置扫描]  Ctrl+C 可提前终止")
    stop = print_status(gripper)
    try:
        positions = [0, 20, 40, 60, 80, 85, 60, 40, 20, 0]
        for pos in positions:
            gripper.set_position(pos, vmax=60.0, fmax=DEFAULT_FMAX)
            time.sleep(1.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        time.sleep(0.1)
    print_status(gripper, once=True)


def _wait_position(gripper, target, timeout=10.0):
    """发送移动指令后轮询等待到位，返回 (ok, last_pos)。"""
    deadline = time.time() + timeout
    last_pos = target
    while time.time() < deadline:
        try:
            status = gripper.get_gripper_status()
            last_pos = status.get('position', last_pos) if isinstance(status, dict) else last_pos
            if abs(last_pos - target) <= DEFAULT_TOL:
                return True, last_pos
        except Exception:
            pass
        time.sleep(0.05)
    return False, last_pos


def test_speed(gripper):
    print("\n[速度测试]  先张开夹爪...")
    gripper.set_position(POS_OPEN, vmax=DEFAULT_VMAX, fmax=DEFAULT_FMAX)
    ok, pos = _wait_position(gripper, POS_OPEN, timeout=8.0)
    print(f"    {'到位 ✓' if ok else f'超时 ✗  停在 {pos:.1f}mm'}")

    for v in [40.0, 80.0, 200.0, 350.0]:
        print(f"  vmax={v:.0f} mm/s → 闭合...")
        t0 = time.time()
        gripper.set_position(POS_CLOSE, vmax=v, fmax=DEFAULT_FMAX)
        ok, pos = _wait_position(gripper, POS_CLOSE, timeout=10.0)
        print(f"    {'到位' if ok else f'超时(停在 {pos:.1f}mm)'}, 用时 {time.time()-t0:.2f}s")

        print(f"  vmax={v:.0f} mm/s → 张开...")
        t0 = time.time()
        gripper.set_position(POS_OPEN, vmax=v, fmax=DEFAULT_FMAX)
        ok, pos = _wait_position(gripper, POS_OPEN, timeout=10.0)
        print(f"    {'到位' if ok else f'超时(停在 {pos:.1f}mm)'}, 用时 {time.time()-t0:.2f}s")
        time.sleep(0.3)


def test_force(gripper):
    print("\n[力控测试]  先张开...")
    gripper.set_position_sync(POS_OPEN, vmax=DEFAULT_VMAX,
                              fmax=DEFAULT_FMAX, tolerance=DEFAULT_TOL, timeout=8.0)

    for f in [5.0, 10.0, 20.0, 40.0]:
        print(f"  fmax={f:.0f} N → 闭合（观察力反馈）...")
        gripper.set_position(POS_CLOSE, vmax=30.0, fmax=f)
        time.sleep(2.0)
        print_status(gripper, once=True)
        print("  → 张开...")
        gripper.set_position_sync(POS_OPEN, vmax=DEFAULT_VMAX,
                                  fmax=DEFAULT_FMAX, tolerance=DEFAULT_TOL, timeout=8.0)
        time.sleep(0.3)


def test_led(gripper):
    print("\n[LED 测试]")
    colors = [("红",255,0,0),("绿",0,255,0),("蓝",0,0,255),("白",255,255,255),("关",0,0,0)]
    try:
        for name, r, g, b in colors:
            print(f"  LED → {name}")
            gripper.set_led_color(r, g, b)
            time.sleep(1.0)
    except AttributeError:
        print("  此版本不支持 LED 控制（串口模式）")


def test_continuous(gripper):
    print("\n[连续往复]  Ctrl+C 停止")
    stop = print_status(gripper)
    try:
        flag = True
        while True:
            pos = POS_OPEN if flag else POS_CLOSE
            gripper.set_position(pos, vmax=DEFAULT_VMAX, fmax=DEFAULT_FMAX)
            flag = not flag
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        time.sleep(0.1)
    print("  停止，回到中间...")
    gripper.set_position_sync(42.5, vmax=DEFAULT_VMAX,
                              fmax=DEFAULT_FMAX, tolerance=DEFAULT_TOL, timeout=8.0)


def test_calibrate(gripper):
    print("\n[标定]")
    print("  请确认夹爪已手动完全张开，然后按 Enter 开始标定...")
    input()
    gripper.calibrate()
    print("  标定完成，当前位置已设为 85mm")
    print_status(gripper, once=True)


# ── 菜单 ─────────────────────────────────────────────────────────────────────

MENU = [
    ("s", "状态读取",   test_status),
    ("o", "开合测试",   test_open_close),
    ("w", "位置扫描",   test_position_sweep),
    ("v", "速度测试",   test_speed),
    ("f", "力控测试",   test_force),
    ("l", "LED 测试",   test_led),
    ("c", "连续往复",   test_continuous),
    ("C", "标定",       test_calibrate),
    ("q", "退出",       None),
]


def run_menu(gripper):
    while True:
        print("\n" + "=" * 50)
        print("  Xense 夹爪测试菜单")
        print("=" * 50)
        for key, desc, _ in MENU:
            print(f"  [{key}] {desc}")
        print("=" * 50)

        choice = input("请选择: ").strip()
        if choice == 'q':
            break

        found = False
        for key, desc, fn in MENU:
            if choice == key and fn:
                found = True
                try:
                    fn(gripper)
                except Exception as e:
                    logger.error(f"{desc} 出错: {e}")
                break
        if not found:
            print("  无效选项")


# ── 主入口 ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Xense 夹爪测试程序")
    parser.add_argument("--arm",  default=DEFAULT_ARM, choices=["left", "right"],
                        help=f"选择夹爪（left=左臂, right=右臂，默认: {DEFAULT_ARM}）")
    parser.add_argument("--mac",  default=None,
                        help="手动指定 MAC 地址（优先于 --arm）")
    parser.add_argument("--port", default=None,
                        help="串口设备（如 /dev/ttyUSB0）")
    parser.add_argument("--test", default=None,
                        choices=[k for k, *_ in MENU if k != 'q'],
                        help="非交互模式：直接运行指定测试项后退出")
    args = parser.parse_args()

    gripper = connect(mac=args.mac, arm=args.arm, port=args.port)

    print("\n初始状态:")
    print_status(gripper, once=True)

    if args.test:
        for key, _, fn in MENU:
            if key == args.test and fn:
                fn(gripper)
                break
    else:
        run_menu(gripper)

    print("\n测试结束，夹爪保持当前位置。")


if __name__ == "__main__":
    main()
