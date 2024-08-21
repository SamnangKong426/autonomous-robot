#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class ImuToOdometryNode(Node):
    def __init__(self):
        super().__init__('imu_to_odometry')

        # IMU data subscriber
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10)
        
        # Odometry publisher
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # tf broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Initialize position and velocity
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.quaternion = [0.0, 0.0, 0.0, 1.0]  # Default quaternion (identity)

        self.last_time = self.get_clock().now()

        # Noise threshold for velocity
        self.velocity_threshold = 0.3

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Update linear velocity with the IMU data
        self.velocity[0] += msg.linear_acceleration.x * dt
        self.velocity[1] += msg.linear_acceleration.y * dt
        self.velocity[2] += msg.linear_acceleration.z * dt

        # Apply threshold to filter out noise before updating position
        if abs(self.velocity[0]) > self.velocity_threshold:
            self.position[0] += self.velocity[0] * dt
        if abs(self.velocity[1]) > self.velocity_threshold:
            self.position[1] += self.velocity[1] * dt

        self.position[2] = 0.0  # Assuming a 2D planar motion, fix Z to 0

        # Update quaternion with the IMU data (already provided as quaternion)
        self.quaternion[0] = msg.orientation.x
        self.quaternion[1] = msg.orientation.y
        self.quaternion[2] = msg.orientation.z
        self.quaternion[3] = msg.orientation.w

        # Publish odometry and broadcast transform
        self.publish_odometry(msg)
        self.broadcast_transform(msg.header.stamp)

    def publish_odometry(self, msg):
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set position
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]

        # Set orientation using the quaternion from the IMU
        odom.pose.pose.orientation.x = self.quaternion[0]
        odom.pose.pose.orientation.y = self.quaternion[1]
        odom.pose.pose.orientation.z = self.quaternion[2]
        odom.pose.pose.orientation.w = self.quaternion[3]

        # Set linear velocity
        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = 0.0  # Assuming 2D movement

        # Set angular velocity (assuming only yaw rate is relevant)
        odom.twist.twist.angular.z = msg.angular_velocity.z

        # Publish odometry
        self.odom_publisher.publish(odom)

    def broadcast_transform(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        # Set translation
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]

        # Set rotation using the quaternion from the IMU
        t.transform.rotation.x = self.quaternion[0]
        t.transform.rotation.y = self.quaternion[1]
        t.transform.rotation.z = self.quaternion[2]
        t.transform.rotation.w = self.quaternion[3]

        # Send the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuToOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
