import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tong',
    maintainer_email='samnangkong499@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_control = robot_odometry.mecanum_control:main',
            # 'odom_node = robot_odometry.mecanum_odom:main',
            'odom_node = robot_odometry.odometry:main',
        ],
    },
)
