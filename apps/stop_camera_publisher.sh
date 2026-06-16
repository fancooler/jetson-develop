#!/bin/bash
# 停止 camera_publisher 节点
#
# 用法：
#   bash ~/work/app/stop_camera_publisher.sh

pids=$(pgrep -f camera_publisher.py || true)
if [ -z "$pids" ]; then
    echo "[stop_camera_publisher] 未找到运行中的 camera_publisher"
    exit 0
fi

echo "[stop_camera_publisher] 终止 PID: $pids"
pkill -f camera_publisher.py
sleep 1

# 再检查一次
remaining=$(pgrep -f camera_publisher.py || true)
if [ -n "$remaining" ]; then
    echo "[stop_camera_publisher] 仍存活，强杀 -9: $remaining"
    pkill -9 -f camera_publisher.py
fi
echo "[stop_camera_publisher] 已停止"
