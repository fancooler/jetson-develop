#!/usr/bin/env python3
"""
test_wrench.py — 双腕六维力实时读取测试

数据来源：SDK subscribe() → outputs[i]['fb_joint_them'][0:6]
  outputs[0] = 左臂，outputs[1] = 右臂
  前6位 = [Fx, Fy, Fz, Mx, My, Mz]，单位 N / N·m

用法：
    cd app/test
    python3 test_wrench.py            # 交互菜单
    python3 test_wrench.py --once     # 单次读取后退出
    python3 test_wrench.py --hz 50    # 指定刷新频率
"""

import sys
import os
import time
import argparse
import logging

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arm_utils import DualArm

logging.basicConfig(
    format='%(asctime)s %(levelname)-5s %(message)s',
    datefmt='%H:%M:%S',
    level=logging.WARNING,   # 测试时屏蔽 arm_utils 的 INFO 刷屏
)
logger = logging.getLogger('test_wrench')

LABELS = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
UNITS  = ['N',  'N',  'N',  'N·m','N·m','N·m']


def _read_wrench(da: DualArm) -> dict | None:
    """单次 subscribe，返回 {'left': [6 floats], 'right': [6 floats]}，失败返回 None。"""
    try:
        data = da._robot.subscribe(da._dcss)
        if not data or len(data.get('outputs', [])) < 2:
            return None
        return {
            'left':  list(data['outputs'][0].get('fb_joint_them', [0]*7))[:6],
            'right': list(data['outputs'][1].get('fb_joint_them', [0]*7))[:6],
        }
    except Exception as e:
        logger.warning(f'subscribe 异常: {e}')
        return None


def _fmt_row(label: str, vals: list) -> str:
    nums = '  '.join(f'{v:>+9.3f}' for v in vals)
    return f'  {label:<8}  {nums}'


def _print_header():
    hdr  = '  '.join(f'{l:>9}' for l in LABELS)
    unit = '  '.join(f'{u:>9}' for u in UNITS)
    print(f'\n  {"":8}  {hdr}')
    print(f'  {"":8}  {unit}')


# ── 测试项 ────────────────────────────────────────────────────────────────────

def test_once(da: DualArm):
    """单次读取并打印。"""
    print('\n[单次读取]')
    _print_header()
    w = _read_wrench(da)
    if w is None:
        print('  读取失败')
        return
    print(_fmt_row('左腕 L', w['left']))
    print(_fmt_row('右腕 R', w['right']))


def test_live(da: DualArm, hz: float = 20.0):
    """实时刷新，Ctrl-C 退出。"""
    print('\n[实时显示]  Ctrl-C 退出\n')
    _print_header()
    n_lines = 3   # left行 + right行 + 状态行
    print('\n' * n_lines, end='')
    interval = 1.0 / hz if hz > 0 else 0.0
    n = 0
    try:
        while True:
            t0 = time.time()
            w = _read_wrench(da)
            lines = []
            if w:
                lines.append(_fmt_row('左腕 L', w['left']))
                lines.append(_fmt_row('右腕 R', w['right']))
                elapsed = time.time() - t0
                lines.append(f'  frame={n}  {1.0/max(elapsed,1e-6):.0f} Hz')
                n += 1
            else:
                lines = ['  读取失败'] + [''] * (n_lines - 1)

            sys.stdout.write(f'\033[{n_lines}A')
            for ln in lines[:n_lines]:
                sys.stdout.write('\033[2K' + ln + '\n')
            sys.stdout.flush()

            elapsed = time.time() - t0
            if interval - elapsed > 0:
                time.sleep(interval - elapsed)
    except KeyboardInterrupt:
        print('\n  已停止')


def test_zero_check(da: DualArm, n: int = 50):
    """采集 n 帧计算静止均值，评估零漂。"""
    print(f'\n[零漂检查]  采集 {n} 帧（请保持机器人静止）...')
    samples = {'left': [], 'right': []}
    for _ in range(n):
        w = _read_wrench(da)
        if w:
            samples['left'].append(w['left'])
            samples['right'].append(w['right'])
        time.sleep(0.02)

    _print_header()
    for side in ('left', 'right'):
        if not samples[side]:
            print(f'  {side}: 无数据')
            continue
        mean = [sum(s[i] for s in samples[side]) / len(samples[side]) for i in range(6)]
        label = '左腕均值' if side == 'left' else '右腕均值'
        print(_fmt_row(label, mean))

    print(f'\n  共采集 left={len(samples["left"])} right={len(samples["right"])} 帧')


def test_load(da: DualArm, hz: float = 20.0):
    """施加负载测试：先记录零点，然后实时显示差值，便于验证 6 轴响应。"""
    print('\n[负载测试]  保持静止，采集零点（1s）...')
    zero = {'left': [0.0]*6, 'right': [0.0]*6}
    n_zero, samples = 0, {'left': [], 'right': []}
    t_end = time.time() + 1.0
    while time.time() < t_end:
        w = _read_wrench(da)
        if w:
            samples['left'].append(w['left'])
            samples['right'].append(w['right'])
            n_zero += 1
        time.sleep(0.02)
    for side in ('left', 'right'):
        if samples[side]:
            zero[side] = [sum(s[i] for s in samples[side]) / len(samples[side]) for i in range(6)]
    print(f'  零点已记录（{n_zero} 帧）。现在施加负载，Ctrl-C 退出。\n')

    _print_header()
    n_lines = 3
    print('\n' * n_lines, end='')
    interval = 1.0 / hz if hz > 0 else 0.0
    n = 0
    try:
        while True:
            t0 = time.time()
            w = _read_wrench(da)
            lines = []
            if w:
                dl = [w['left'][i]  - zero['left'][i]  for i in range(6)]
                dr = [w['right'][i] - zero['right'][i] for i in range(6)]
                lines.append(_fmt_row('左腕 ΔL', dl))
                lines.append(_fmt_row('右腕 ΔR', dr))
                lines.append(f'  frame={n}  (相对零点差值)')
                n += 1
            else:
                lines = ['  读取失败'] + [''] * (n_lines - 1)
            sys.stdout.write(f'\033[{n_lines}A')
            for ln in lines[:n_lines]:
                sys.stdout.write('\033[2K' + ln + '\n')
            sys.stdout.flush()
            elapsed = time.time() - t0
            if interval - elapsed > 0:
                time.sleep(interval - elapsed)
    except KeyboardInterrupt:
        print('\n  已停止')


# ── 菜单 ─────────────────────────────────────────────────────────────────────

MENU = [
    ('s', '单次读取',   lambda da, hz: test_once(da)),
    ('l', '实时显示',   lambda da, hz: test_live(da, hz)),
    ('z', '零漂检查',   lambda da, hz: test_zero_check(da)),
    ('L', '负载测试',   lambda da, hz: test_load(da, hz)),
    ('q', '退出',       None),
]


def run_menu(da: DualArm, hz: float):
    while True:
        print('\n' + '=' * 50)
        print('  六维力传感器测试菜单')
        print('=' * 50)
        for key, desc, _ in MENU:
            print(f'  [{key}] {desc}')
        print('=' * 50)
        choice = input('请选择: ').strip()
        if choice == 'q':
            break
        for key, desc, fn in MENU:
            if choice == key and fn:
                try:
                    fn(da, hz)
                except Exception as e:
                    logger.error(f'{desc} 出错: {e}')
                break
        else:
            print('  无效选项')


# ── 主入口 ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='双腕六维力测试')
    parser.add_argument('--once', action='store_true', help='单次读取后退出')
    parser.add_argument('--hz',   type=float, default=20.0, help='刷新频率 Hz（默认 20）')
    args = parser.parse_args()

    print('连接机械臂...')
    da = DualArm()
    if not da.connect():
        print('连接失败')
        sys.exit(1)
    print('连接成功\n')

    try:
        if args.once:
            test_once(da)
        else:
            test_once(da)   # 先打印一次，确认数据正常
            run_menu(da, args.hz)
    finally:
        da.release()
        print('已断开')


if __name__ == '__main__':
    main()
