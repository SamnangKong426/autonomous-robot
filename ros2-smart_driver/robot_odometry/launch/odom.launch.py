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
    
    # smart_driver = Node(package='can_motor', executable='can_motor_node', output='screen')

    ros_imu = Node(package='ros_imu', executable='ros_imu_node', output='screen')
    odom_node = Node(package='robot_odometry', executable='odom_node', output='screen')


    # Launch them all!
    return LaunchDescription([
        # smart_driver,
        ros_imu,
        odom_node
    ])