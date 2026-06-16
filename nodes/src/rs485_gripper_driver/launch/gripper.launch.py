import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 声明参数（可以在命令行中通过 port:=/dev/ttyUSB1 这种方式修改）
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')
    baud_arg = DeclareLaunchArgument('baud', default_value='921600')
    device_id_arg = DeclareLaunchArgument('device_id', default_value='1')
    rate_arg = DeclareLaunchArgument('query_rate', default_value='50.0')

    # 2. 定义节点
    gripper_node = Node(
        package='rs485_gripper_driver',      # package.xml 里的名字
        executable='gripper_service',  # setup.py 里的 entry_point 名字
        name='gripper_rs485_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'device_id': LaunchConfiguration('device_id'),
            'query_rate': LaunchConfiguration('query_rate'),
        }]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        device_id_arg,
        rate_arg,
        gripper_node
    ])