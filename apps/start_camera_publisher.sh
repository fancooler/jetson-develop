#!/bin/bash
# 启动 camera_publisher 节点到后台
#
# 用法（在 Jetson 上直接跑，或者从 ThinkBook 用 ssh 跑）：
#   bash ~/work/app/start_camera_publisher.sh
#
# 日志：~/work/app/log/camera_publisher_YYYYMMDD_HHMMSS.log
# 停止：bash ~/work/app/stop_camera_publisher.sh

set -e

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$APP_DIR/log"
mkdir -p "$LOG_DIR"
# 固定文件名 + 覆盖（>），每次启动只保留最近一次、不再累积带时间戳的旧日志。
# 配合 publisher 已去掉周期 stats、告警限流，本文件稳态几乎不增长。
# 若想彻底不落盘，把下面的 "$LOG_FILE" 改成 /dev/null 即可。
LOG_FILE="$LOG_DIR/camera_publisher.log"

# 杀掉可能还在跑的旧实例（避免抢占 RealSense 设备）
pkill -f camera_publisher.py 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
cd "$APP_DIR"

nohup python3 -u camera_publisher.py > "$LOG_FILE" 2>&1 &
disown
PID=$!

echo "[start_camera_publisher] PID=$PID"
echo "[start_camera_publisher] log=$LOG_FILE （覆盖式，不再累积）"
