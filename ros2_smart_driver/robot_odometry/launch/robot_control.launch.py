import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # rsp = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','rsp.launch.py'
    #             )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    # )
    
    smart_driver = Node(package='can_motor', executable='motor_sender_node', output='screen')

    mecanum_control = Node(package='robot_odometry', executable='mecanum_control', output='screen')

    ros_imu = Node(package='ros_imu', executable='ros_imu_node', output='screen')

    # Launch them all!
    return LaunchDescription([
        ros_imu,
        smart_driver,
        mecanum_control
    ])