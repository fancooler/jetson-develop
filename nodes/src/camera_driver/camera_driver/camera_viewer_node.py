"""camera_viewer_node.py — 订阅三路压缩图像并以田字格显示

订阅 camera_publisher_node 发布的 JPEG 压缩 topic（省带宽，适合远端查看），
解码后拼成田字格用 cv2.imshow 显示：

  ┌────────────┬────────────┐
  │   front    │  (unused)  │
  ├────────────┼────────────┤
  │ left_wrist │ right_wrist│
  └────────────┴────────────┘

按 q / ESC 退出。

前提：
  - camera_publisher_node 已在（本机或远端 Jetson）运行；
  - viewer 端与发布端 ROS_DOMAIN_ID 一致、能 DDS 互相发现；
  - 需要图形界面（cv2.imshow，无头 SSH 不行）。

运行：
  ros2 run camera_driver camera_viewer
"""

import threading

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage

from camera_driver import topics


WINDOW_NAME = "Triple Camera (front | (empty) / left_wrist | right_wrist)"


class CameraViewerNode(Node):
    """三路压缩图像订阅 + 锁保护最新帧缓存。spin 在后台线程，主线程做 cv2.imshow。"""

    def __init__(self):
        super().__init__('camera_viewer')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.width  = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._lock   = threading.Lock()
        self._latest = {r: None for r in topics.ROLES}
        self._subs   = []

        for role in topics.ROLES:
            sub = self.create_subscription(
                CompressedImage,
                topics.TOPICS_COMPRESSED_BY_ROLE[role],
                self._make_callback(role),
                qos,
            )
            self._subs.append(sub)
            self.get_logger().info(
                f"[sub/{role}] {topics.TOPICS_COMPRESSED_BY_ROLE[role]}"
            )

    def _make_callback(self, role: str):
        def cb(msg: CompressedImage):
            try:
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is None:
                    self.get_logger().warning(f"[sub/{role}] JPEG 解码失败")
                    return
            except Exception as e:
                self.get_logger().warning(f"[sub/{role}] 解码异常: {e}")
                return
            with self._lock:
                self._latest[role] = bgr
        return cb

    def get_latest(self) -> dict:
        with self._lock:
            return {r: (None if f is None else f.copy())
                    for r, f in self._latest.items()}


def _placeholder(text: str, w: int, h: int) -> np.ndarray:
    """生成一张黑底带文字的占位帧（用于尚未收到帧的窗格）。"""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    cv2.putText(img, text, (40, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (90, 90, 90), 2)
    return img


def _annotate(img: np.ndarray, label: str) -> np.ndarray:
    """左上角加 topic 标签（不改原图，返回副本）。"""
    out = img.copy()
    cv2.rectangle(out, (0, 0), (260, 28), (0, 0, 0), -1)
    cv2.putText(out, label, (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1)
    return out


def _make_grid(latest: dict, w: int, h: int) -> np.ndarray:
    """田字格拼接：左上 front, 右上空, 左下 left_wrist, 右下 right_wrist。"""
    front = (latest['front']       if latest['front']       is not None
             else _placeholder("front: waiting ...", w, h))
    lw    = (latest['left_wrist']  if latest['left_wrist']  is not None
             else _placeholder("left_wrist: waiting ...", w, h))
    rw    = (latest['right_wrist'] if latest['right_wrist'] is not None
             else _placeholder("right_wrist: waiting ...", w, h))
    empty = _placeholder("(unused)", w, h)

    front = _annotate(front, "front")
    lw    = _annotate(lw,    "left_wrist")
    rw    = _annotate(rw,    "right_wrist")

    top    = np.hstack([front, empty])
    bottom = np.hstack([lw,    rw])
    return np.vstack([top, bottom])


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()
    w, h = node.width, node.height

    # spin 在后台线程，主线程做 cv2 显示（cv2.imshow 必须在主线程）
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    node.get_logger().info("窗口已打开，按 q 退出 ...")

    try:
        while True:
            latest = node.get_latest()
            grid = _make_grid(latest, w, h)
            cv2.imshow(WINDOW_NAME, grid)

            # 33ms ≈ 30Hz 刷新；同时处理 GUI 事件
            key = cv2.waitKey(33) & 0xFF
            if key == ord('q') or key == 27:  # q 或 ESC
                break
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C，正在退出 ...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
