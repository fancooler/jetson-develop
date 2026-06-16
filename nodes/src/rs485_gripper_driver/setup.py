from setuptools import setup
import os
from glob import glob

package_name = 'rs485_gripper_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 包含 launch 文件夹
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    description='Gripper Driver',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 这里的 gripper_service 是可执行文件名
            'gripper_service = rs485_gripper_driver.gripper_node:main'
        ],
    },
)