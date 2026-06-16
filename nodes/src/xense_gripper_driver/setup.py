from setuptools import setup
import os
from glob import glob

package_name = 'xense_gripper_driver'

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
    description='Xense 双夹爪 ROS2 驱动（薄包装 gripper.py，实现 gripper_interfaces）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gripper_node = xense_gripper_driver.gripper_node:main',
        ],
    },
)
