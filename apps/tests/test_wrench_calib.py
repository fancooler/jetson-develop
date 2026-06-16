#!/usr/bin/env python3
"""
test_wrench_calib.py — 六维力传感器零偏一致性测试

流程（共 10 轮）：
  1. 提示放零负载 → 采样 N 帧 → 计算均值
  2. 提示挂固定负载 → 采样 N 帧 → 计算均值
  3. 记录差值（零负载 - 固定负载）
  4. 10 轮结束后统计差值的均值和标准差，评估一致性

运行（在 ROS_DOMAIN_ID=2 环境下）：
  python3 test_wrench_calib.py
  python3 test_wrench_calib.py --rounds 5 --samples 50 --arm left
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

LABELS = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']


def wrench_to_list(w):
    return [w.force.x, w.force.y, w.force.z,
            w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z]


class WrenchCollector(Node):
    def __init__(self, arm: str):
        super().__init__(f'wrench_calib_{arm}')
        self._buf = []
        self._lock = threading.Lock()
        self._collecting = False
        topic = f'/arm/wrench_{arm}'
        self.create_subscription(WrenchStamped, topic, self._cb, 10)
        self.get_logger().info(f'订阅 {topic}')

    def _cb(self, msg: WrenchStamped):
        if not self._collecting:
            return
        w = msg.wrench
        with self._lock:
            self._buf.append([w.force.x,  w.force.y,  w.force.z,
                               w.torque.x, w.torque.y, w.torque.z])

    def collect(self, n: int, timeout: float = 10.0) -> list:
        """采样 n 帧，返回各通道均值列表。"""
        with self._lock:
            self._buf.clear()
        self._collecting = True

        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                cnt = len(self._buf)
            if cnt >= n:
                break
            time.sleep(0.02)

        self._collecting = False
        with self._lock:
            data = list(self._buf[:n])

        if len(data) < n:
            print(f'  ⚠️  只收到 {len(data)}/{n} 帧（超时），用已有数据计算')

        if not data:
            return [0.0] * 6
        means = [sum(row[i] for row in data) / len(data) for i in range(6)]
        return means


def vec_sub(a, b):
    return [x - y for x, y in zip(a, b)]


def stats(vecs):
    """输入 list of list，返回每通道 (mean, std)。"""
    n = len(vecs)
    means = [sum(v[i] for v in vecs) / n for i in range(6)]
    stds  = [math.sqrt(sum((v[i] - means[i])**2 for v in vecs) / n) for i in range(6)]
    return means, stds


def fmt_row(label, vals, fmt='.1f'):
    return f'  {label:>12s}: ' + '  '.join(f'{v:{fmt}}' for v in vals)


def run(arm: str, rounds: int, samples: int):
    rclpy.init()
    node = WrenchCollector(arm)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    time.sleep(0.5)  # 等订阅建立

    zero_means_list = []
    load_means_list = []
    diff_list        = []

    print(f'\n=== 六维力标定测试  臂={arm}  轮数={rounds}  每轮采样={samples} ===\n')

    for r in range(1, rounds + 1):
        print(f'─── 第 {r}/{rounds} 轮 ───')

        input('  [零负载] 确认无负载后按 Enter 开始采样...')
        print(f'  采样中（{samples} 帧）...', end='', flush=True)
        z = node.collect(samples)
        print(' 完成')
        print(fmt_row('零负载均值', z))

        input('  [固定负载] 挂上负载后按 Enter 开始采样...')
        print(f'  采样中（{samples} 帧）...', end='', flush=True)
        l = node.collect(samples)
        print(' 完成')
        print(fmt_row('固定负载均值', l))

        diff = vec_sub(z, l)
        print(fmt_row('差值 (零-负载)', diff))
        print()

        zero_means_list.append(z)
        load_means_list.append(l)
        diff_list.append(diff)

    # ── 汇总统计 ──────────────────────────────────────────────────────────────
    print('═' * 60)
    print(f'  汇总（{rounds} 轮）')
    print('═' * 60)
    print(f'  {"通道":>6}  ' + '  '.join(f'{lb:>8}' for lb in LABELS))

    z_mean, z_std = stats(zero_means_list)
    l_mean, l_std = stats(load_means_list)
    d_mean, d_std = stats(diff_list)

    print('\n  零负载均值（各轮平均）:')
    print('  ' + '  '.join(f'{lb:>8}: {v:9.1f}' for lb, v in zip(LABELS, z_mean)))
    print('  零负载标准差（轮间波动）:')
    print('  ' + '  '.join(f'{lb:>8}: {v:9.1f}' for lb, v in zip(LABELS, z_std)))

    print('\n  固定负载均值:')
    print('  ' + '  '.join(f'{lb:>8}: {v:9.1f}' for lb, v in zip(LABELS, l_mean)))

    print('\n  差值均值 (零-负载):')
    print('  ' + '  '.join(f'{lb:>8}: {v:9.1f}' for lb, v in zip(LABELS, d_mean)))
    print('  差值标准差（一致性，越小越好）:')
    print('  ' + '  '.join(f'{lb:>8}: {v:9.1f}' for lb, v in zip(LABELS, d_std)))

    print('\n  一致性评价（差值标准差 / 差值均值 × 100%）:')
    for lb, dm, ds in zip(LABELS, d_mean, d_std):
        if abs(dm) > 1e-6:
            cv = abs(ds / dm) * 100
            mark = '✅' if cv < 5 else ('⚠️ ' if cv < 15 else '❌')
            print(f'    {lb:>4}: {mark} {cv:.1f}%')
        else:
            print(f'    {lb:>4}: （差值近零，跳过）')

    print()
    node.destroy_node()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description='六维力零偏一致性测试')
    ap.add_argument('--arm',     default='left',  choices=['left', 'right'])
    ap.add_argument('--rounds',  type=int, default=10, help='测试轮数（默认10）')
    ap.add_argument('--samples', type=int, default=30, help='每次采样帧数（默认30）')
    args = ap.parse_args()
    run(args.arm, args.rounds, args.samples)


if __name__ == '__main__':
    main()
