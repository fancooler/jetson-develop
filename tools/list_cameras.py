#!/usr/bin/env python3
"""list_cameras.py — 列出当前连接的所有 RealSense 相机及序列号

用法：
    python3 tools/list_cameras.py

输出示例：
    检测到 3 台设备：

    序号  序列号          型号                      固件版本
    ----  --------------  ------------------------  -----------
    1     254622074992    Intel RealSense D435      5.16.0.1
    2     260322273418    Intel RealSense D405      5.16.0.1
    3     260322272642    Intel RealSense D405      5.16.0.1

    提示：拔插 USB 线缆可区分左腕/右腕序列号
"""

import sys

try:
    import pyrealsense2 as rs
except ImportError:
    print("错误：未找到 pyrealsense2")
    print("请先部署 thirdparty/camera/ 下的 .so 文件（见 docs/onboarding.md Step 2.1）")
    sys.exit(1)


def main():
    ctx = rs.context()
    devices = list(ctx.devices)

    if not devices:
        print("未检测到任何 RealSense 设备")
        print("请检查：USB 是否插好（需 USB3）；是否被其他进程占用（pkill -f camera_node）")
        sys.exit(1)

    print(f"\n检测到 {len(devices)} 台设备：\n")
    print(f"{'序号':<4}  {'序列号':<14}  {'型号':<24}  {'固件版本'}")
    print(f"{'----':<4}  {'----------':<14}  {'------------------------':<24}  {'--------'}")

    for i, dev in enumerate(devices, 1):
        sn   = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)
        fw   = dev.get_info(rs.camera_info.firmware_version)
        print(f"{i:<4}  {sn:<14}  {name:<24}  {fw}")

    print()
    print("提示：拔插 USB 线缆可区分左腕/右腕序列号")
    print("      将序列号填入 config/robots.yaml 对应机器人的 cameras 字段")


if __name__ == "__main__":
    main()
