import os
# from launch.actions import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    # rsp = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(""),'launch','rsp.launch.py'
    #             )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    # )

    ros_imu = Node(
        package="ros_imu",
        executable="ros_imu_node",
    )

    imu_to_odom = Node(
        package="autonomous_robot",
        executable="imu_to_odom_node",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory('autonomous_robot'), 'cfg', 'imu_odom.rviz')],
        output="screen",
    )
    

    return LaunchDescription([
        ros_imu,
        imu_to_odom,
        rviz,
    ])