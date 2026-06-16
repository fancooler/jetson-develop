"""camera_publisher.py — 三路 RealSense → ROS2 image_raw

启动后独占三路 RealSense 硬件，按 30Hz 拉帧并发布到：
  /camera_front/color/image_raw
  /camera_left_wrist/color/image_raw
  /camera_right_wrist/color/image_raw

消息：sensor_msgs/Image，BGR8（与 OpenCV 原生格式一致）
QoS：BestEffort + depth=1（订阅端只关心最新帧，丢帧无所谓）

运行：
  source /opt/ros/humble/setup.bash
  cd ~/work/app
  python3 camera_publisher.py
"""

import os
import sys
import time
import threading
import logging

import numpy as np
import cv2
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import config_camera as cam_cfg
import camera_topics as topics


logging.basicConfig(
    format='%(asctime)s %(levelname)-5s %(name)s: %(message)s',
    datefmt='%H:%M:%S', level=logging.INFO,
)
logger = logging.getLogger("camera_publisher")


class CameraPublisherNode(Node):
    """三路 RealSense 拉帧 + 发布。每路一个独立后台线程。"""

    def __init__(self):
        super().__init__('camera_publisher')

        # BestEffort + depth=1 是相机流的标准 QoS，丢帧不重传
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._bridge       = CvBridge()
        self._pubs         = {}
        self._pubs_compr   = {}
        self._pipes        = {}
        self._running      = False
        self._last_warn    = {}   # role -> 上次告警时间戳，用于限流
        self._jpeg_params  = [int(cv2.IMWRITE_JPEG_QUALITY), topics.JPEG_QUALITY]

        for role in topics.ROLES:
            self._pubs[role] = self.create_publisher(
                Image, topics.TOPICS_BY_ROLE[role], qos
            )
            self._pubs_compr[role] = self.create_publisher(
                CompressedImage, topics.TOPICS_COMPRESSED_BY_ROLE[role], qos
            )

        self._serials = {
            'front':       cam_cfg.HEAD_CAM_SERIAL,
            'left_wrist':  cam_cfg.WRIST_L_CAM_SERIAL,
            'right_wrist': cam_cfg.WRIST_R_CAM_SERIAL,
        }

    # ── 相机生命周期 ─────────────────────────────────────────────────────────

    def start_cameras(self):
        for role, serial in self._serials.items():
            pipe = rs.pipeline()
            cfg  = rs.config()
            if serial:
                cfg.enable_device(serial)
            cfg.enable_stream(
                rs.stream.color,
                cam_cfg.CAM_WIDTH, cam_cfg.CAM_HEIGHT,
                rs.format.bgr8, cam_cfg.CAM_FPS,
            )
            try:
                pipe.start(cfg)
                self._pipes[role] = pipe
                logger.info(f"[cam/{role}] 已启动  serial={serial}  "
                            f"topic={topics.TOPICS_BY_ROLE[role]}")
            except Exception as e:
                logger.error(
                    f"[cam/{role}] 启动失败: {e}\n"
                    f"  若有其他进程占用，先 pkill -f camera_publisher 或 pkill -f python3"
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
        logger.info("[cam] 全部已关闭")

    # ── 后台线程 ─────────────────────────────────────────────────────────────

    def _reader(self, role: str):
        pipe      = self._pipes[role]
        pub       = self._pubs[role]
        pub_compr = self._pubs_compr[role]
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
                ok, enc = cv2.imencode('.jpg', bgr, self._jpeg_params)
                if ok:
                    cmsg = CompressedImage()
                    cmsg.header.stamp    = stamp
                    cmsg.header.frame_id = frame_id
                    cmsg.format          = 'jpeg'
                    cmsg.data            = enc.tobytes()
                    pub_compr.publish(cmsg)
            except Exception as e:
                # 限流：相机掉线时本循环会高速空转，不限流会瞬间写爆磁盘。
                # 同一路最多每 30s 告警一条，并 sleep 让出 CPU、避免狂转。
                if self._running:
                    now = time.time()
                    if now - self._last_warn.get(role, 0.0) > 30.0:
                        logger.warning(f"[cam/{role}] 读帧/发布异常: {e}")
                        self._last_warn[role] = now
                    time.sleep(0.2)


def main():
    rclpy.init()
    node = CameraPublisherNode()
    try:
        node.start_cameras()
        logger.info("发布中，Ctrl+C 退出 ...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Ctrl+C，正在退出 ...")
    finally:
        node.stop_cameras()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
