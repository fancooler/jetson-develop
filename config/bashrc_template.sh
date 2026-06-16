#!/usr/bin/env bash
# bashrc_template.sh — 每台 Jetson ~/.bashrc 末尾需追加的内容
#
# 装机时执行（替换 ROBOT_ID 和 DEVELOP_DIR 后追加）：
#   ROBOT_ID=robot2 DEVELOP_DIR=~/develop \
#     bash -c 'sed "s/__ROBOT_ID__/$ROBOT_ID/g; s|__DEVELOP_DIR__|$DEVELOP_DIR|g" \
#     ~/develop/config/bashrc_template.sh >> ~/.bashrc'
#
# 或直接 cat 查看后手动粘贴进 ~/.bashrc。

# ── ROS2 ──────────────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source __DEVELOP_DIR__/nodes/install/setup.bash

# ── CUDA ──────────────────────────────────────────────────────────────────────
export PATH=/usr/local/cuda/bin:$PATH

# ── 第三方库 ──────────────────────────────────────────────────────────────────
export QT_API=pyside6

# ── 机器人身份（每台机器唯一，装机时修改）────────────────────────────────────
export ROBOT_ID=__ROBOT_ID__

# ── 从 robots.yaml 自动推导 ROS_DOMAIN_ID 和 ROBOTS_YAML ─────────────────────
# robot_env.sh 读取 ROBOT_ID，从 config/robots.yaml 查 domain_id 并 export
source __DEVELOP_DIR__/config/robot_env.sh
