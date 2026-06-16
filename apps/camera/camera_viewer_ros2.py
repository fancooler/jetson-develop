#!/usr/bin/env python3
"""
camera_viewer_ros2.py — 纯 ROS2 三路相机田字格查看器【SDK 工具 / 调试】

在你自己的机器上订阅机器人三路相机话题并实时显示（确认相机链路 / 看机器人视角）。
只用 ROS2 标准消息（sensor_msgs），**不依赖 SDK 自定义包，也不依赖天机/Xense/realsense SDK**。
因此连 SDK 的 colcon 工作区都不用 source，只要 ROS2 Humble + opencv-python 即可。

═══ 运行前提 ═══
  - 机器人 Jetson 在跑 camera_driver：ros2 launch camera_driver camera.launch.py
  - 本机 ROS2 Humble（ros-humble-desktop 自带 cv_bridge）+ opencv-python，
    且 ROS_DOMAIN_ID 与机器人一致（如 robot1 → 1）：
      source /opt/ros/humble/setup.bash
      export ROS_DOMAIN_ID=1
  - 显示窗口需要桌面/X11；headless 服务器（无显示）用 --snapshot 存图代替开窗。

═══ 用法 ═══
  python3 camera_viewer_ros2.py                  # 订阅原始图（同机/局域网）
  python3 camera_viewer_ros2.py --compressed     # 跨 WiFi 用压缩流（省带宽，推荐）
  python3 camera_viewer_ros2.py --snapshot ./snaps   # 不开窗：每路各存一帧后退出（headless）
  窗口内按 q 或 ESC 退出。
"""

import argparse
import os
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

# 与 camera_driver / camera_topics.py 对齐；内联以便独立分发
ROLES = ('front', 'left_wrist', 'right_wrist')
CAM_TOPICS = {
    'front':       '/camera_front/color/image_raw',
    'left_wrist':  '/camera_left_wrist/color/image_raw',
    'right_wrist': '/camera_right_wrist/color/image_raw',
}

# 与 camera_driver 一致：BestEffort + 只留最新一帧
IMG_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=1,
                     durability=DurabilityPolicy.VOLATILE)


class CameraViewerSub(Node):
    """订阅三路相机，后台 spin，get(role) 取最新帧（BGR，非阻塞）。"""

    def __init__(self, compressed: bool = False):
        super().__init__('sdk_camera_viewer')
        self._compressed = compressed
        self._bridge = CvBridge()
        self._latest = {r: None for r in ROLES}
        self._locks = {r: threading.Lock() for r in ROLES}
        for role in ROLES:
            topic = CAM_TOPICS[role] + ('/compressed' if compressed else '')
            msg_type = CompressedImage if compressed else Image
            self.create_subscription(msg_type, topic, self._make_cb(role), IMG_QOS)

    def _make_cb(self, role: str):
        def cb(msg):
            try:
                if self._compressed:
                    arr = np.frombuffer(msg.data, np.uint8)
                    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                else:
                    bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:                      # noqa: BLE001
                self.get_logger().warn(f'[{role}] 解码失败: {e}', throttle_duration_sec=5.0)
                return
            with self._locks[role]:
                self._latest[role] = bgr
        return cb

    def get(self, role: str):
        with self._locks[role]:
            f = self._latest[role]
        return None if f is None else f.copy()

    def have_any(self) -> bool:
        return any(self._latest[r] is not None for r in ROLES)


def _cell(img, w: int, h: int, label: str):
    """缩放一帧到 (w,h) 并叠角色名；无帧给灰底占位（cv2 字体只画 ASCII）。"""
    if img is None:
        cell = np.full((h, w, 3), 40, np.uint8)
        cv2.putText(cell, f'{label}: no frame', (10, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        return cell
    cell = cv2.resize(img, (w, h))
    cv2.putText(cell, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return cell


def compose_grid(sub: CameraViewerSub, cw: int, ch: int):
    """2x2 田字格：front | left_wrist / right_wrist | info。"""
    front = _cell(sub.get('front'),       cw, ch, 'front')
    lw    = _cell(sub.get('left_wrist'),  cw, ch, 'left_wrist')
    rw    = _cell(sub.get('right_wrist'), cw, ch, 'right_wrist')
    info  = np.full((ch, cw, 3), 25, np.uint8)
    cv2.putText(info, 'q / ESC : quit', (10, ch // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2)
    return np.vstack([np.hstack([front, lw]), np.hstack([rw, info])])


def main():
    ap = argparse.ArgumentParser(description='纯 ROS2 三路相机田字格查看器')
    ap.add_argument('--compressed', action='store_true',
                    help='订阅压缩流(/compressed)，跨 WiFi 省带宽（推荐）')
    ap.add_argument('--rate', type=float, default=30.0, help='刷新率 Hz（默认 30）')
    ap.add_argument('--cell', type=int, default=480, help='每格宽度像素（默认 480，高按 3:4）')
    ap.add_argument('--snapshot', metavar='DIR',
                    help='不开窗：等首帧后每路各存一张 PNG 到 DIR 然后退出（headless 用）')
    ap.add_argument('--wait', type=float, default=8.0, help='等首帧超时(秒)')
    args = ap.parse_args()
    cw = args.cell
    ch = int(cw * 3 / 4)

    rclpy.init()
    sub = CameraViewerSub(compressed=args.compressed)

    def _spin():
        try:
            rclpy.spin(sub)
        except Exception:
            pass
    spin = threading.Thread(target=_spin, daemon=True)
    spin.start()

    print(f'订阅三路相机（{"压缩" if args.compressed else "原始"}流），等待首帧 ...')
    deadline = time.time() + args.wait
    while time.time() < deadline and not sub.have_any():
        time.sleep(0.1)
    if not sub.have_any():
        hint = ('压缩流可能没发布（去掉 --compressed 试原始）' if args.compressed
                else '跨 WiFi 建议加 --compressed')
        print(f'⚠️ 超时未收到任何相机帧。检查：camera_driver 在跑？ROS_DOMAIN_ID 一致？{hint}。'
              '（仍会开窗显示占位，收到帧会自动刷新）')

    try:
        if args.snapshot:
            os.makedirs(args.snapshot, exist_ok=True)
            n = 0
            for r in ROLES:
                f = sub.get(r)
                if f is not None:
                    p = os.path.join(args.snapshot, f'{r}.png')
                    cv2.imwrite(p, f)
                    n += 1
                    print(f'  存 {p}')
                else:
                    print(f'  跳过 {r}（无帧）')
            print(f'共存 {n} 张到 {args.snapshot}。')
        else:
            period_ms = max(1, int(1000.0 / max(1.0, args.rate)))
            win = 'robot cameras (front | left_wrist / right_wrist)'
            while rclpy.ok():
                grid = compose_grid(sub, cw, ch)
                try:
                    cv2.imshow(win, grid)
                except cv2.error as e:
                    print(f'⚠️ 无法开窗口（headless 无显示？）: {e}\n   改用 --snapshot 存图。')
                    break
                if (cv2.waitKey(period_ms) & 0xFF) in (ord('q'), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        if spin.is_alive():
            spin.join(timeout=2.0)
        print('查看器退出。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
