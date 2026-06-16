"""tj_marvin.launch.py — 启动天机双臂 ROS2 驱动节点

默认 use_mock=true（不碰真机）。接真机：
    ros2 launch tj_marvin_driver tj_marvin.launch.py use_mock:=false

用 OpaqueFunction 在启动时把 launch 参数解析成正确的 Python 类型
（bool/float），避免 LaunchConfiguration 字符串注入 ROS 参数时的类型坑。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)
    use_mock = lc('use_mock').strip().lower() in ('1', 'true', 'yes', 'on')
    app_dir = os.path.expanduser(lc('app_dir'))
    auto_connect = lc('auto_connect').strip().lower() in ('1', 'true', 'yes', 'on')
    publish_rate = float(lc('publish_rate'))

    node = Node(
        package='tj_marvin_driver',
        executable='arm_node',
        name='tj_marvin_arm',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_mock': use_mock,
            'app_dir': app_dir,
            'auto_connect': auto_connect,
            'publish_rate': publish_rate,
        }],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='true',
                              description='true=MockDualArm(不碰真机)；false=真机'),
        DeclareLaunchArgument('app_dir', default_value='~/work/app',
                              description='jetson-work 的 app 目录（含 arm_utils/config_dual）'),
        DeclareLaunchArgument('auto_connect', default_value='true',
                              description='启动即 connect（仅设控制模式，不运动）'),
        DeclareLaunchArgument('publish_rate', default_value='25.0',
                              description='状态发布频率 Hz'),
        OpaqueFunction(function=_setup),
    ])
