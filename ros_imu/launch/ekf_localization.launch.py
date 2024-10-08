from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('ros_imu'))
    ekf_cfg = os.path.join(pkg_path,'config','ekf.yaml')

    return LaunchDescription([
        Node(
            package="ros_imu",
            executable="ros_imu_node",
            name="ros_imu",
            output="screen",    
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[ekf_cfg]
        ),
    ])