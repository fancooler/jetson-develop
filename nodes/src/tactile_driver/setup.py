from setuptools import setup
import os
from glob import glob

package_name = 'tactile_driver'

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
    description='Xense 视触觉传感器 ROS2 驱动节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tactile_node = tactile_driver.tactile_node:main',
        ],
    },
)
