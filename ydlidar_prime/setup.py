from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ydlidar_prime'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='primerobotics',
    maintainer_email='emmanuelprime2@gmail.com',
    description='YDLidar ROS2 Python driver using ydlidar SDK',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ydlidar_node = ydlidar_prime.ydlidar_node:main',
        ],
    },
)
