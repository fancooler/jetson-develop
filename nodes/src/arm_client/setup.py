from setuptools import setup

package_name = 'arm_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lvyong',
    maintainer_email='lvyongsh@msn.com',
    description='通用机械臂客户端封装（ArmClient 类 + arm_cli 命令行，对着 /arm/* + arm_interfaces）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_cli = arm_client.cli:main',
        ],
    },
)
