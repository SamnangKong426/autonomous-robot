#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_messages.msg import MotorFeedback
from nav_msgs.msg import Odometry
import math

class MecanumOdometryNode(Node):
    def __init__(self):
        super().__init__('mecanum_odometry_node')
        
        # Parameters
        self.declare_parameter('wheel_base_x', 0.48)  # meters, distance between front and rear wheels
        self.declare_parameter('wheel_base_y', 0.4)  # meters, distance between left and right wheels
        self.declare_parameter('wheel_diameter', 0.15)  # meters
        self.declare_parameter('cpr', 19.2 * 13)  # counts per revolution
        
        self.wheel_base_x = self.get_parameter('wheel_base_x').get_parameter_value().double_value
        self.wheel_base_y = self.get_parameter('wheel_base_y').get_parameter_value().double_value
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.cpr = self.get_parameter('cpr').get_parameter_value().integer_value  # Updated here

        self.encoder_fl = 0  # Front Left
        self.encoder_fr = 0  # Front Right
        self.encoder_bl = 0  # Back Left
        self.encoder_br = 0  # Back Right

        # Publishers and Subscribers
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.encoder_subscriber = self.create_subscription(
            MotorFeedback,
            'motor_feedback',
            self.encoder_callback,
            10
        )
        
        # Timer to publish odometry at a regular interval
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def encoder_callback(self, msg):
        self.encoder_fl = msg.position[0]
        self.encoder_fr = msg.position[1]
        self.encoder_bl = msg.position[2]
        self.encoder_br = msg.position[3]

    def timer_callback(self):
        # Calculate wheel circumference
        circumference = self.wheel_diameter * math.pi
        
        # Compute distances for each wheel
        distance_fl = (self.encoder_fl / self.cpr) * circumference
        distance_fr = (self.encoder_fr / self.cpr) * circumference
        distance_bl = (self.encoder_bl / self.cpr) * circumference
        distance_br = (self.encoder_br / self.cpr) * circumference

        # Average distances for odometry
        d_left = (distance_fl + distance_bl) / 2
        d_right = (distance_fr + distance_br) / 2
        
        d = (d_left + d_right) / 2
        theta_change = (d_right - d_left) / self.wheel_base_x

        # Update pose
        self.x += d * math.cos(self.theta + theta_change / 2)
        self.y += d * math.sin(self.theta + theta_change / 2)
        self.theta += theta_change

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
