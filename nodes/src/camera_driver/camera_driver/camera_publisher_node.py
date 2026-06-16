"""camera_publisher_node.py — 三路 RealSense → ROS2 image_raw

独占三路 RealSense（头部 D435 + 左/右腕 D405），按固定帧率拉帧并发布到
（topic 名见 topics.py，对齐官方 realsense2_camera 命名）：
  /camera_front/color/image_raw
  /camera_left_wrist/color/image_raw
  /camera_right_wrist/color/image_raw
以及对应的 .../compressed（JPEG，省带宽，远端 viewer 用）。

消息：sensor_msgs/Image，BGR8（与 OpenCV 原生格式一致）
QoS：BestEffort + depth=1（订阅端只关心最新帧，丢帧无所谓）

硬件序列号 / 分辨率 / 帧率 / JPEG 质量等通过 ROS2 参数配置，
默认值见 launch/camera.launch.py。

运行：
  ros2 launch camera_driver camera.launch.py
或：
  ros2 run camera_driver camera_node
"""

import time
import threading

import numpy as np
import cv2
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from camera_driver import topics


class CameraPublisherNode(Node):
    """三路 RealSense 拉帧 + 发布。每路一个独立后台线程。"""

    def __init__(self):
        super().__init__('camera_publisher')

        # ── 参数声明 ─────────────────────────────────────────────────────────
        # 序列号默认值仅作兜底（robot1）；正常由 camera.launch.py 从
        # config/robots.yaml 按 ROBOT_ID 注入，多机器人无需改这里。
        self.declare_parameter('head_serial',        '254622074992')  # D435 头部（前向）
        self.declare_parameter('left_wrist_serial',  '260322273418')  # D405 左腕
        self.declare_parameter('right_wrist_serial', '260322272642')  # D405 右腕
        self.declare_parameter('width',              640)
        self.declare_parameter('height',             480)
        self.declare_parameter('fps',                30)
        self.declare_parameter('jpeg_quality',       80)
        self.declare_parameter('publish_compressed', True)

        gp = self.get_parameter
        self._width  = gp('width').get_parameter_value().integer_value
        self._height = gp('height').get_parameter_value().integer_value
        self._fps    = gp('fps').get_parameter_value().integer_value
        self._publish_compressed = gp('publish_compressed').get_parameter_value().bool_value
        jpeg_quality = gp('jpeg_quality').get_parameter_value().integer_value
        self._jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]

        self._serials = {
            'front':       gp('head_serial').get_parameter_value().string_value,
            'left_wrist':  gp('left_wrist_serial').get_parameter_value().string_value,
            'right_wrist': gp('right_wrist_serial').get_parameter_value().string_value,
        }

        # ── ROS 接口 ──────────────────────────────────────────────────────────
        # BestEffort + depth=1 是相机流的标准 QoS，丢帧不重传
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._bridge     = CvBridge()
        self._pubs       = {}
        self._pubs_compr = {}
        self._pipes      = {}
        self._running    = False
        self._last_warn  = {}   # role -> 上次告警时间戳，用于限流

        for role in topics.ROLES:
            self._pubs[role] = self.create_publisher(
                Image, topics.TOPICS_BY_ROLE[role], qos
            )
            if self._publish_compressed:
                self._pubs_compr[role] = self.create_publisher(
                    CompressedImage, topics.TOPICS_COMPRESSED_BY_ROLE[role], qos
                )

    # ── 相机生命周期 ─────────────────────────────────────────────────────────

    def start_cameras(self):
        for role, serial in self._serials.items():
            pipe = rs.pipeline()
            cfg  = rs.config()
            if serial:
                cfg.enable_device(serial)
            cfg.enable_stream(
                rs.stream.color,
                self._width, self._height,
                rs.format.bgr8, self._fps,
            )
            try:
                pipe.start(cfg)
                self._pipes[role] = pipe
                self.get_logger().info(
                    f"[cam/{role}] 已启动  serial={serial}  "
                    f"topic={topics.TOPICS_BY_ROLE[role]}"
                )
            except Exception as e:
                self.get_logger().error(
                    f"[cam/{role}] 启动失败: {e}  "
                    f"（若被占用，先停掉其他相机进程：pkill -f camera_node）"
                )

        if not self._pipes:
            raise RuntimeError("所有相机启动失败，退出")

        self._running = True
        for role in self._pipes:
            threading.Thread(
                target=self._reader, args=(role,), daemon=True
            ).start()

    def stop_cameras(self):
        self._running = False
        for role, pipe in self._pipes.items():
            try:
                pipe.stop()
            except Exception:
                pass
        self.get_logger().info("[cam] 全部已关闭")

    # ── 后台线程 ─────────────────────────────────────────────────────────────

    def _reader(self, role: str):
        pipe      = self._pipes[role]
        pub       = self._pubs[role]
        pub_compr = self._pubs_compr.get(role)
        frame_id  = topics.FRAME_IDS_BY_ROLE[role]

        while self._running:
            try:
                frames = pipe.wait_for_frames(timeout_ms=1000)
                color  = frames.get_color_frame()
                if not color:
                    continue
                bgr = np.asanyarray(color.get_data())

                stamp = self.get_clock().now().to_msg()

                # 原始 BGR8（本机 runner 订阅，带宽充足）
                msg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
                msg.header.stamp    = stamp
                msg.header.frame_id = frame_id
                pub.publish(msg)

                # JPEG 压缩（远端 viewer 订阅，省带宽）
                if pub_compr is not None:
                    ok, enc = cv2.imencode('.jpg', bgr, self._jpeg_params)
                    if ok:
                        cmsg = CompressedImage()
                        cmsg.header.stamp    = stamp
                        cmsg.header.frame_id = frame_id
                        cmsg.format          = 'jpeg'
                        cmsg.data            = enc.tobytes()
                        pub_compr.publish(cmsg)
            except Exception as e:
                # 限流：相机掉线时本循环会高速空转，不限流会瞬间写爆日志。
                # 同一路最多每 30s 告警一条，并 sleep 让出 CPU、避免狂转。
                if self._running:
                    now = time.time()
                    if now - self._last_warn.get(role, 0.0) > 30.0:
                        self.get_logger().warning(f"[cam/{role}] 读帧/发布异常: {e}")
                        self._last_warn[role] = now
                    time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        node.start_cameras()
        node.get_logger().info("发布中，Ctrl+C 退出 ...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C，正在退出 ...")
    finally:
        node.stop_cameras()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
