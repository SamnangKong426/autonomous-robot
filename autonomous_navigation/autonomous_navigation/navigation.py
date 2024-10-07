#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from transforms3d.euler import quat2euler
import math as m
import time
# from simple_pid import PID
from ultralytics import YOLO

class AutonomousNavigation(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        self.subscription_camera = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10)
        self.subscription_imu = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.isStarted = False
        self.started_roll = 0.0
        self.robot_angle = 0.0
        self.odom = [0.0, 0.0, self.robot_angle]
        # self.pid = PID(0.5, 0.2, 0.05, setpoint=0.0, output_limits=(-5.0, 5.0))

        # Object detection
        self.model = YOLO("yolov8n-seg.pt")  # load an official segmentation model



    def camera_callback(self, msg):
        # Process camera data
        results = self.model.track(source="https://youtu.be/LNwODJXcvt4", show=True, tracker="bytetrack.yaml")



    def imu_callback(self, msg):
        # Process IMU data
        roll, pitch, yaw = quat2euler([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        # self.get_logger().info('Roll: %f, Pitch: %f, Yaw: %f' % (m.degrees(roll), m.degrees(pitch), m.degrees(yaw)))
        if not self.isStarted:
            self.isStarted = True
            self.started_roll = roll
        else:
            time.sleep(0.1)
            self.normalize_angle(roll)
            self.robot_movement(-90, 0, 90)

    def normalize_angle(self, roll):
        # Calculate the robot's orientation
        self.robot_angle = roll - self.started_roll
        self.robot_angle = m.degrees((self.robot_angle + m.pi) % (2 * m.pi) - m.pi)
        self.get_logger().info('Gamma: %f' % self.robot_angle)

    def local_vel_to_global_vel(self, vx, vy, gamma):
        # Convert local velocity to global velocity
        VxG = (vx * m.cos(m.radians(gamma))) - (vy * m.sin(m.radians(gamma)))
        VyG = (vx * m.sin(m.radians(gamma))) + (vy * m.cos(m.radians(gamma)))

        return VxG, VyG

    def robot_movement(self, vx, vy, omega):
        vx, vy = self.local_vel_to_global_vel(vx, vy, omega)
        omega = (omega - self.robot_angle)
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(omega)
        self.publisher_.publish(twist)
        self.get_logger().info('Vx: %f, Vy: %f, Omega: %f' % (twist.linear.x, twist.linear.y, twist.angular.z))


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()