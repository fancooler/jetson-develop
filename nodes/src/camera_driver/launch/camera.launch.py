"""camera.launch.py — 多机器人感知的相机发布启动文件

行为：
  1. 读环境变量 ROBOT_ID（默认 'robot1'）；
  2. 从车队注册表 robots.yaml 按 ROBOT_ID 取该机的 3 个相机序列号；
     注册表路径由 $ROBOTS_YAML 提供（仓库顶层 config/robot_env.sh 导出）；
  3. 校验当前 ROS_DOMAIN_ID 是否与注册表里该机 domain_id 一致，不一致时醒目告警；
  4. 把序列号注入 camera_node。

加新机器人无需改本文件，只改 <repo>/config/robots.yaml + 目标机 ROBOT_ID。
以其它机器人配置启动： ROBOT_ID=robot2 ros2 launch camera_driver camera.launch.py
（需先 source <repo>/config/robot_env.sh，.bashrc 通常已配）
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def _registry_path():
    """定位车队注册表 robots.yaml：用 $ROBOTS_YAML（仓库顶层 config/robot_env.sh 导出）。"""
    path = os.environ.get('ROBOTS_YAML')
    if not path:
        raise RuntimeError(
            "未设 $ROBOTS_YAML。车队注册表在仓库顶层 config/robots.yaml，由 "
            "config/robot_env.sh 导出该变量。请新开终端（.bashrc 通常已 source 它），"
            "或手动 export ROBOTS_YAML=<repo>/config/robots.yaml。")
    if not os.path.exists(path):
        raise RuntimeError(f"$ROBOTS_YAML={path} 指向的文件不存在。")
    return path


def _load_robot_config(robot_id):
    """从中央注册表读取指定机器人的配置（缺失则抛出清晰错误）。"""
    reg_path = _registry_path()
    with open(reg_path, 'r') as f:
        registry = yaml.safe_load(f) or {}
    robots = registry.get('robots', {})
    if robot_id not in robots:
        raise RuntimeError(
            f"robots.yaml 里没有 ROBOT_ID='{robot_id}' 的配置；"
            f"已知机器人: {sorted(robots)}。请在 {reg_path} 添加该机器人。")
    return robots[robot_id]


def generate_launch_description():
    robot_id = os.environ.get('ROBOT_ID', 'robot1')
    cfg = _load_robot_config(robot_id)
    cams = cfg['cameras']
    expected_domain = cfg.get('domain_id')

    actions = []

    # 校验 ROS_DOMAIN_ID 与注册表一致（防止忘 source robot_env.sh 导致串台/收不到）
    try:
        actual_domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    except ValueError:
        actual_domain = None
    if expected_domain is not None and actual_domain != expected_domain:
        actions.append(LogInfo(msg=(
            f"⚠️  [camera_driver] ROS_DOMAIN_ID={actual_domain} 与 {robot_id} "
            f"注册表 domain_id={expected_domain} 不一致！"
            f" 请先: export ROBOT_ID={robot_id} && source .../config/robot_env.sh")))

    actions.append(LogInfo(msg=(
        f"[camera_driver] ROBOT_ID={robot_id}  domain_id={expected_domain}  "
        f"head={cams['head_serial']} L={cams['left_wrist_serial']} "
        f"R={cams['right_wrist_serial']}")))

    actions.append(Node(
        package='camera_driver',
        executable='camera_node',
        name='camera_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # 序列号来自 robots.yaml（已是字符串，类型正确，无需 ParameterValue 转换）
            'head_serial':        cams['head_serial'],
            'left_wrist_serial':  cams['left_wrist_serial'],
            'right_wrist_serial': cams['right_wrist_serial'],
            # 以下对所有机器人相同，按训练数据固定
            'width':              640,
            'height':             480,
            'fps':                30,
            'jpeg_quality':       80,
            'publish_compressed': True,
        }],
    ))

    return LaunchDescription(actions)
