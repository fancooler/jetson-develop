"""tactile.launch.py — 视触觉传感器节点启动文件"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration as lc
from launch_ros.actions import Node


def _load_macs():
    robot_id = os.environ.get('ROBOT_ID', 'robot1')
    path = os.environ.get('ROBOTS_YAML', '')
    if path and os.path.exists(path):
        with open(path) as f:
            reg = yaml.safe_load(f) or {}
        grippers = reg.get('robots', {}).get(robot_id, {}).get('grippers', {})
        return grippers.get('left_mac', ''), grippers.get('right_mac', ''), robot_id
    return '', '', robot_id


def generate_launch_description():
    left_mac, right_mac, robot_id = _load_macs()

    return LaunchDescription([
        DeclareLaunchArgument('app_dir',      default_value=os.path.expanduser('~/develop/drivers')),
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('use_gpu',      default_value='true'),
        DeclareLaunchArgument('left_mac',     default_value=left_mac),
        DeclareLaunchArgument('right_mac',    default_value=right_mac),

        LogInfo(msg=f"[tactile_driver] ROBOT_ID={robot_id} left={left_mac} right={right_mac}"),

        Node(
            package='tactile_driver',
            executable='tactile_node',
            name='tactile_driver',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'app_dir':      lc('app_dir'),
                'publish_rate': lc('publish_rate'),
                'use_gpu':      lc('use_gpu'),
                'left_mac':     lc('left_mac'),
                'right_mac':    lc('right_mac'),
            }],
        ),
    ])
