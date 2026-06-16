#!/usr/bin/env python3
"""list_cameras.py — 列出/预览当前连接的 RealSense 相机，读取序列号。

换相机后用它读出新序列号，再填进 config/robots.yaml，然后 colcon build + 重启。
只依赖 pyrealsense2（预览模式额外要 opencv），不依赖 ROS，可在起节点前直接跑。

用法（在 Jetson 上、插好所有相机后）：
    python3 list_cameras.py                      # 列出所有设备（型号/序列号/USB/固件）
    ros2 run camera_driver list_cameras          # 同上（需先 colcon build）
    python3 list_cameras.py --preview <序列号>    # 预览某台的彩色流，按 q 退出

左右腕区分：D435=头部(唯一好认)；两台 D405 型号相同、序列号本身分不出左右，
用 --preview <某D405序列号> 对着某个腕相机晃手，看哪个窗口在动来确认它是左腕还是右腕。
（历史上就因为左右填反过，见 git 提交 b950e85。）
"""
import argparse
import sys

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit("✗ 未找到 pyrealsense2。Jetson 上应已装；其它机器：pip install pyrealsense2")


def _info(dev, key, default='?'):
    try:
        return dev.get_info(key)
    except Exception:
        return default


def list_devices():
    devs = list(rs.context().query_devices())
    if not devs:
        print("✗ 没检测到 RealSense 设备。检查 USB 连接 / 供电 / 线缆（D405 对供电敏感）。")
        return
    print(f"检测到 {len(devs)} 台 RealSense：\n")
    print(f"{'#':<3}{'型号 Name':<26}{'序列号 Serial':<16}{'USB':<8}{'固件 FW':<14}建议角色")
    print("-" * 86)
    for i, d in enumerate(devs):
        name   = _info(d, rs.camera_info.name)
        serial = _info(d, rs.camera_info.serial_number)
        fw     = _info(d, rs.camera_info.firmware_version)
        usb    = _info(d, rs.camera_info.usb_type_descriptor)
        if 'D435' in name:
            role = 'head 头部'
        elif 'D405' in name:
            role = 'wrist 腕（左/右用 --preview 区分）'
        else:
            role = '?'
        print(f"{i:<3}{name:<26}{serial:<16}{usb:<8}{fw:<14}{role}")
    print()
    print("下一步：把序列号填进 <repo>/config/robots.yaml 对应机器人的")
    print("        head_serial / left_wrist_serial / right_wrist_serial，再 colcon build + 重启。")
    print("两台 D405 分不清左右：python3 list_cameras.py --preview <序列号>，对左腕晃手看哪个窗口动。")


def preview(serial):
    try:
        import cv2
        import numpy as np
    except ImportError:
        sys.exit("✗ 预览需要 opencv + numpy（Jetson 上应已装）。")
    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    try:
        pipe.start(cfg)
    except Exception as e:
        sys.exit(f"✗ 打不开 serial={serial}: {e}\n（被占用？先 pkill -f camera_node）")
    print(f"预览 {serial} … 对着它晃手确认是左腕还是右腕，按 q 退出。")
    win = f"RealSense {serial}  (q 退出)"
    try:
        while True:
            c = pipe.wait_for_frames().get_color_frame()
            if not c:
                continue
            img = np.asanyarray(c.get_data())
            cv2.putText(img, serial, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow(win, img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipe.stop()
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description="列出/预览 RealSense 相机，读取序列号")
    ap.add_argument('--preview', metavar='SERIAL',
                    help="预览指定序列号的彩色流（用于区分左右腕 D405）")
    args = ap.parse_args()
    if args.preview:
        preview(args.preview)
    else:
        list_devices()


if __name__ == '__main__':
    main()
