from setuptools import setup
import os
from glob import glob

package_name = 'camera_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 包含 launch 文件夹
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 注：车队注册表已移到仓库顶层 config/（robots.yaml + robot_env.sh），
        # 不再随本包安装；camera.launch.py 经 $ROBOTS_YAML 读取。
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lvyong',
    maintainer_email='lvyongsh@msn.com',
    description='三路 RealSense → ROS2 image_raw 发布节点（头部 D435 + 左/右腕 D405）',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 可执行文件名（ros2 run camera_driver <name>）
            'camera_node   = camera_driver.camera_publisher_node:main',
            'camera_viewer = camera_driver.camera_viewer_node:main',
            'list_cameras  = camera_driver.list_cameras:main',
        ],
    },
)
