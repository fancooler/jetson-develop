import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_path',
        default_value='',
        description='gateway_config.yaml 路径；空则按优先级查找（GATEWAY_CONFIG 环境变量 → 包内默认）',
    )

    node = Node(
        package='mqtt_gateway',
        executable='mqtt_gateway',
        name='mqtt_gateway',
        output='screen',
        parameters=[{'config_path': LaunchConfiguration('config_path')}],
    )

    return LaunchDescription([config_arg, node])
