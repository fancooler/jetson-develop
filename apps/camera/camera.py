"""双路 RealSense 摄像头封装（D415 头部 + D405 腕部）

后台线程持续读帧，主循环调 read() 只取最新缓存帧，不会因控制循环暂停而超时。
"""
import threading
import numpy as np
import logging
import pyrealsense2 as rs

import config

logger = logging.getLogger(__name__)

_BLANK = {
    'head':  None,
    'wrist': None,
}


class DualCamera:
    """
    同时管理头部（D415）和腕部（D405）两路 RealSense 摄像头。
    每路摄像头有独立后台线程持续拉帧，read() 返回最新缓存帧（非阻塞）。
    """

    def __init__(self):
        self._pipes   = {}
        self._latest  = {'head': None, 'wrist': None}   # 最新 RGB 帧
        self._locks   = {'head': threading.Lock(), 'wrist': threading.Lock()}
        self._running = False
        self._threads = {}

    def start(self):
        serials = self._find_serials()
        for role, serial in serials.items():
            pipe = rs.pipeline()
            cfg  = rs.config()
            if serial:
                cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.color,
                              config.CAM_WIDTH, config.CAM_HEIGHT,
                              rs.format.bgr8, config.CAM_FPS)
            try:
                pipe.start(cfg)
                self._pipes[role] = pipe
                logger.info(f"摄像头 [{role}] 已启动"
                            + (f"  serial={serial}" if serial else ""))
            except Exception as e:
                logger.error(f"摄像头 [{role}] 启动失败（serial={serial}）: {e}"
                             f"\n  请检查是否有其他进程占用该摄像头（pkill -f python3）")

        # 预热：等待第一帧就绪
        self._running = True
        for role in self._pipes:
            t = threading.Thread(target=self._reader, args=(role,), daemon=True)
            t.start()
            self._threads[role] = t

        # 等两路都拿到第一帧再返回
        import time
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if all(self._latest[r] is not None for r in self._pipes):
                break
            time.sleep(0.01)
        else:
            logger.warning("等待首帧超时，部分摄像头可能未就绪")

    def read(self):
        """
        返回 (head_rgb, wrist_rgb)，均为 (H, W, 3) uint8 RGB numpy array。
        非阻塞，立即返回最新缓存帧；若尚未有帧则返回空帧。
        """
        head  = self._get('head')
        wrist = self._get('wrist')
        return head, wrist

    def close(self):
        self._running = False
        for role, pipe in self._pipes.items():
            try:
                pipe.stop()
            except Exception:
                pass
            logger.info(f"摄像头 [{role}] 已关闭")

    # ── 内部方法 ──────────────────────────────────────────────────────────────

    def _get(self, role: str) -> np.ndarray:
        with self._locks[role]:
            frame = self._latest.get(role)
        if frame is None:
            return np.zeros((config.CAM_HEIGHT, config.CAM_WIDTH, 3), dtype=np.uint8)
        return frame

    def _reader(self, role: str):
        """后台线程：持续从摄像头拉帧并缓存最新帧。"""
        pipe = self._pipes[role]
        while self._running:
            try:
                frames = pipe.wait_for_frames(timeout_ms=1000)
                color  = frames.get_color_frame()
                if not color:
                    continue
                img_bgr = np.asanyarray(color.get_data())
                img_rgb = img_bgr[:, :, ::-1].copy()
                with self._locks[role]:
                    self._latest[role] = img_rgb
            except Exception as e:
                if self._running:
                    logger.warning(f"摄像头 [{role}] 读帧异常: {e}")

    def _find_serials(self) -> dict:
        head_serial  = config.HEAD_CAM_SERIAL
        wrist_serial = config.WRIST_CAM_SERIAL

        if head_serial is None or wrist_serial is None:
            ctx     = rs.context()
            devices = ctx.query_devices()
            serials = [d.get_info(rs.camera_info.serial_number) for d in devices]
            logger.info(f"发现 RealSense 设备: {serials}")
            if head_serial is None:
                head_serial  = serials[0] if len(serials) > 0 else None
            if wrist_serial is None:
                wrist_serial = serials[1] if len(serials) > 1 else None

        return {'head': head_serial, 'wrist': wrist_serial}
