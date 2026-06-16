"""xense_gripper.launch.py — 启动 Xense 双夹爪 ROS2 驱动节点

夹爪 MAC 从 $ROBOTS_YAML 注册表按 $ROBOT_ID 读取，无需手动传参。
默认两爪都 mock（不碰硬件）。接真机：
    ros2 launch xense_gripper_driver xense_gripper.launch.py mock_left:=false mock_right:=false
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _b(v):
    return str(v).strip().lower() in ('1', 'true', 'yes', 'on')


def _gripper_macs_from_registry():
    """从 $ROBOTS_YAML 按 $ROBOT_ID 读取左右爪 MAC，失败时抛 RuntimeError。"""
    path = os.environ.get('ROBOTS_YAML')
    rid  = os.environ.get('ROBOT_ID')
    if not path or not rid:
        raise RuntimeError(
            '未设 $ROBOTS_YAML 或 $ROBOT_ID，无法从注册表读取夹爪 MAC。'
            '请确认已 source <repo>/config/robot_env.sh。'
        )
    if not os.path.exists(path):
        raise RuntimeError(f'$ROBOTS_YAML={path} 文件不存在。')
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    robot = data.get('robots', {}).get(rid)
    if robot is None:
        raise RuntimeError(f'robots.yaml 中找不到 ROBOT_ID={rid!r}。')
    grippers = robot.get('grippers')
    if not grippers:
        raise RuntimeError(f'robots.yaml 中 {rid}.grippers 未配置。')
    return str(grippers['left_mac']), str(grippers['right_mac'])


def _setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)

    mac_left, mac_right = _gripper_macs_from_registry()

    node = Node(
        package='xense_gripper_driver',
        executable='gripper_node',
        name='xense_gripper',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'mock_left':    _b(lc('mock_left')),
            'mock_right':   _b(lc('mock_right')),
            'app_dir':      os.path.expanduser(lc('app_dir')),
            'auto_connect': _b(lc('auto_connect')),
            'publish_rate': float(lc('publish_rate')),
            'mac_left':     mac_left,
            'mac_right':    mac_right,
        }],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mock_left',  default_value='true',
                              description='true=左爪 MockGripper(不碰硬件)'),
        DeclareLaunchArgument('mock_right', default_value='true',
                              description='true=右爪 MockGripper(不碰硬件)'),
        DeclareLaunchArgument('app_dir',    default_value='~/work/app',
                              description='含 gripper.py 的 app 目录'),
        DeclareLaunchArgument('auto_connect',  default_value='true'),
        DeclareLaunchArgument('publish_rate',  default_value='25.0'),
        OpaqueFunction(function=_setup),
    ])
