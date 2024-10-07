# autonomous-robot

## Introduction
This repository contains the code for the autonomous robot project. Which is a project for research purposes, to develop a robot that can navigate autonomously in an indoor environment and  outdoor. 

## Software Architecture
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10 (pre-installed in Ubuntu 22.04)

## Hardware
- Jetson orin nano
- Realsense L515
- ROS IMU (HFI-A9)
- Mecanum wheels robot

## Download necessary packages
<details>
  <summary>
    Install latest Intel® RealSense™ SDK 2.0 (LibRealSense v2.51.1)
  </summary>

- [LibRealSense](https://github.com/IntelRealSense/librealsense/releases/tag/v2.51.1)

    #### Here are documentation to build librealsense2 SDK:
    
    ```bash
    $ git clone -b v2.51.1 https://github.com/IntelRealSense/librealsense.git
    $ cd librealsense
    $ mkdir build && cd build
    $ cmake ../ -DBUILD_EXAMPLES=true
    $ sudo make uninstall && make clean && make && sudo make install
    ```


</details>

<details>
  <summary>
    Install ROS2 Realsense package (realsense-ros-4.51.1)
  </summary>

- [Realsense ROS2](https://github.com/IntelRealSense/realsense-ros/releases/tag/4.51.1)
    
    #### Here are the commands to install the realsense-ros package:

    - Create workspace
    ```bash
    $ mkdir -p ~/realsense_ws/src
    $ cd ~/realsense_ws/src
    ```

    - Download realsense-ros source code 
    ```bash
    $ wget https://github.com/IntelRealSense/realsense-ros/archive/refs/tags/4.51.1.tar.gz
    $ tar -xzf realsense-ros-4.51.1.tar.gz
    $ rm -r realsense-ros-4.51.1.tar.gz
    ```
    
    - Build workspace
    ```bash
    cd ~/realsense_ws
    colon build && source install/setup.bash
    ```




</details>


## Installation
- Create a ROS2 workspace
    ```bash
    $ mkdir -p ~/autonomous-robot
    $ cd ~/autonomous-robot
    $ git clone https://github.com/SamnangKong426/autonomous-robot.git src/
    ```
- Build & Source environment
    ```bash
    $ colcon build --symlink-install && source install/setup.bash
    ```


## Packages and Their Purposes

1. **`realsense_camera`**:
   - **Purpose**: Handles the integration and data streaming from the Intel RealSense D455 camera. This package includes launch files to start the camera node and configure its parameters.

2. **`ros_imu`**:
   - **Purpose**: Manages the IMU sensor data. This package includes nodes and launch files for publishing IMU data, transforming it, and integrating it with other sensor data for localization.

3. **`mecanum_robot`**:
   - **Purpose**: Controls the mecanum wheels of the robot. This package includes nodes that subscribe to velocity commands and publish motor commands to control the robot's movement in all directions.

4. **`autonomous_navigation`**:
   - **Purpose**: Implements the autonomous navigation logic. This package processes sensor data from the camera and IMU to generate velocity commands for the robot, enabling it to navigate autonomously.
