import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tong',
    maintainer_email='samnangkong499@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'ros_imu_node = ros_imu.ros2_imu:main',
             'imu_tf2_broadcaster = ros_imu.imu_tf2_broadcaster:main',
        ],
    },
)
