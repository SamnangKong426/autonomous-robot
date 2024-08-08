# ROS2_IMU

## Publishe URDF model
```bash 
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/tong/ros2_ws/src/ros_imu/urdf/imu.urdf.xacro)"
```

```bash
rviz2
```

Change Fixed Frame to base_link and add RobotModel (topic: /robot_description)
    






