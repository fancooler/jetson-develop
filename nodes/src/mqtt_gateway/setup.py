import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mqtt_gateway'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools', 'paho-mqtt', 'pyyaml'],
    zip_safe=True,
    maintainer='lvyong',
    maintainer_email='lvyongsh@msn.com',
    entry_points={
        'console_scripts': [
            'mqtt_gateway = mqtt_gateway.gateway_node:main',
        ],
    },
)
