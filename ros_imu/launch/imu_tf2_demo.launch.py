from launch.substitutions import Command
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro


def generate_launch_description():
    use_sim_time = True
    pkg_path = os.path.join(get_package_share_directory('ros_imu'))
    rviz_config_file = os.path.join(pkg_path, 'config', 'imu.rviz')

    urdf_file = FindPackageShare('ros_imu').find('ros_imu') + '/urdf/imu.urdf.xacro'
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', urdf_file])}]
    )

    ros_imu = Node(
            package='ros_imu',
            executable='ros_imu_node',
            name='sim'
        )
    
    imu_tf2_broadcaster = Node(
            package='ros_imu',
            executable='imu_tf2_broadcaster',
            name='broadcaster',
        )
    
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        )

    return LaunchDescription([
        ros_imu,
        imu_tf2_broadcaster,
        node_robot_state_publisher,
        rviz2
    ])