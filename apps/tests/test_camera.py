#!/usr/bin/env python3
"""摄像头测试脚本 — 枚举设备、抓帧、保存图片"""
import sys
import os
import time
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    print("ERROR: pyrealsense2 未安装，请先 pip install pyrealsense2")
    sys.exit(1)

try:
    import cv2
except ImportError:
    print("ERROR: opencv-python 未安装，请先 pip install opencv-python")
    sys.exit(1)

import config

SAVE_DIR = "/tmp/cam_test"
os.makedirs(SAVE_DIR, exist_ok=True)


def enumerate_devices():
    ctx     = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("未发现任何 RealSense 设备，请检查 USB 连接")
        return []

    print(f"\n发现 {len(devices)} 个 RealSense 设备：")
    serials = []
    for i, dev in enumerate(devices):
        name   = dev.get_info(rs.camera_info.name)
        serial = dev.get_info(rs.camera_info.serial_number)
        usb    = dev.get_info(rs.camera_info.usb_type_descriptor)
        print(f"  [{i}] {name}  serial={serial}  USB={usb}")
        serials.append(serial)
    return serials


def test_single(serial: str, label: str,
                width=config.CAM_WIDTH, height=config.CAM_HEIGHT, fps=config.CAM_FPS):
    print(f"\n── 测试 [{label}]  serial={serial} ──")
    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    try:
        profile = pipe.start(cfg)
    except Exception as e:
        print(f"  启动失败: {e}")
        return False

    # 丢弃前几帧（自动曝光稳定）
    for _ in range(10):
        pipe.wait_for_frames()

    # 抓 3 帧，测量帧间隔
    timestamps = []
    frames_bgr = []
    for i in range(3):
        t0     = time.time()
        frames = pipe.wait_for_frames(timeout_ms=3000)
        color  = frames.get_color_frame()
        timestamps.append(time.time() - t0)
        frames_bgr.append(np.asanyarray(color.get_data()).copy())

    pipe.stop()

    avg_ms = sum(timestamps) / len(timestamps) * 1000
    print(f"  分辨率: {frames_bgr[0].shape[1]}x{frames_bgr[0].shape[0]}")
    print(f"  平均帧获取时间: {avg_ms:.1f}ms")

    # 保存最后一帧
    save_path = os.path.join(SAVE_DIR, f"{label}.jpg")
    cv2.imwrite(save_path, frames_bgr[-1])
    print(f"  已保存: {save_path}")
    return True


def test_dual(serial_head: str, serial_wrist: str,
              width=config.CAM_WIDTH, height=config.CAM_HEIGHT, fps=config.CAM_FPS):
    print(f"\n── 双路同步测试 ──")
    pipe_h = rs.pipeline()
    pipe_w = rs.pipeline()

    cfg_h = rs.config()
    cfg_h.enable_device(serial_head)
    cfg_h.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    cfg_w = rs.config()
    cfg_w.enable_device(serial_wrist)
    cfg_w.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    try:
        pipe_h.start(cfg_h)
        pipe_w.start(cfg_w)
    except Exception as e:
        print(f"  启动失败: {e}")
        return

    for _ in range(10):
        pipe_h.wait_for_frames()
        pipe_w.wait_for_frames()

    # 测量双路同步抓帧耗时
    t0 = time.time()
    for _ in range(5):
        fh = pipe_h.wait_for_frames(timeout_ms=3000)
        fw = pipe_w.wait_for_frames(timeout_ms=3000)
    elapsed_ms = (time.time() - t0) / 5 * 1000
    print(f"  双路平均抓帧耗时: {elapsed_ms:.1f}ms/次")

    img_h = np.asanyarray(fh.get_color_frame().get_data()).copy()
    img_w = np.asanyarray(fw.get_color_frame().get_data()).copy()

    pipe_h.stop()
    pipe_w.stop()

    # 拼接保存
    combined = np.concatenate([img_h, img_w], axis=1)
    save_path = os.path.join(SAVE_DIR, "dual.jpg")
    cv2.imwrite(save_path, combined)
    print(f"  双路拼接图已保存: {save_path}")


def main():
    print("=" * 50)
    print("  RealSense 双摄像头测试")
    print("=" * 50)

    serials = enumerate_devices()
    if not serials:
        return

    if len(serials) < 2:
        print(f"\n警告：只发现 {len(serials)} 个设备，需要 2 个（D415头部 + D405腕部）")

    # 单独测试每个设备
    for i, serial in enumerate(serials):
        label = f"cam{i}_serial{serial}"
        test_single(serial, label)

    # 双路同步测试（至少有 2 个设备时）
    if len(serials) >= 2:
        test_dual(serials[0], serials[1])

    print(f"\n测试完成，图片保存在 {SAVE_DIR}/")
    print("提示：将两个设备的序列号填入 config.py 的 HEAD_CAM_SERIAL / WRIST_CAM_SERIAL")


if __name__ == "__main__":
    main()
