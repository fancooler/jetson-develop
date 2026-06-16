from setuptools import setup
import os
from glob import glob

package_name = 'tj_marvin_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lvyong',
    maintainer_email='lvyongsh@msn.com',
    description='天机 MaRVIN 双臂 ROS2 驱动（薄包装 DualArm，实现 arm_interfaces）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_node = tj_marvin_driver.arm_node:main',
        ],
    },
)
