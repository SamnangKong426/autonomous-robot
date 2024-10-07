import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    ros_imu = Node(package='ros_imu', executable='ros_imu_node', output='screen')
    nav_node = Node(package='autonomous_navigation', executable='nav_node', output='screen')
    mecanum_node = Node(package='mecanum_robot', executable='mecanum_control_node', output='screen')
    smart_driver = Node(package='can_motor', executable='motor_sender_node', output='screen')

    # Launch them all!
    return LaunchDescription([
        smart_driver,
        ros_imu,
        mecanum_node,
        nav_node
    ])