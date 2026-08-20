from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'icp_localization_node = lidar_odom.icp_localization_node:main',
            'cmd_robot = lidar_odom.cmd_robot:main',
            'logger = lidar_odom.logger:main',
            'icp_mapping = lidar_odom.icp_mapping:main',
            'map_loader = lidar_odom.map_loader:main',
            'record_lidar = lidar_odom.record_lidar:main',
            'filter_lidar = lidar_odom.filter_lidar:main',
            'logger_map = lidar_odom.logger_map:main',
        ],
    },
)
