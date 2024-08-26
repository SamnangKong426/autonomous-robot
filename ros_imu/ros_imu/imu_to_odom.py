#!/
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import time

class OdomPredictor(Node):
    def __init__(self):
        super().__init__('odom_predictor')
        
        self.declare_parameter('max_imu_queue_length', 1000)
        self.max_imu_queue_length = self.get_parameter('max_imu_queue_length').get_parameter_value().integer_value
        
        self.seq = 0
        self.have_odom = False
        self.have_bias = False
        
        self.imu_queue = []
        self.estimate_timestamp = None
        self.has_imu_meas = False
        self.frame_id = "world"
        self.child_frame_id = "odom"
        
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.transform = np.eye(4)  # 4x4 identity matrix for transformation
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/uav/sensors/imu',
            self.imu_callback,
            10
        )
        
        self.imu_bias_sub = self.create_subscription(
            Imu,
            'imu_bias',
            self.imu_bias_callback,
            10
        )
        
        self.odom_pub = self.create_publisher(Odometry, 'imu_odometry', 10)
        self.transform_pub = self.create_publisher(TransformStamped, 'imu_transform', 10)
        self.br = TransformBroadcaster(self)

    def imu_callback(self, msg):
        if msg.orientation_covariance[0] == -1.0:
            self.have_orientation = False
        else:
            self.orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.transform[:3, :3] = self.quaternion_to_matrix(self.orientation)[:3, :3]

        if len(self.imu_queue) > 0 and msg.header.stamp < self.imu_queue[-1].header.stamp:
            self.get_logger().error(f"Latest IMU message occurred at time: {msg.header.stamp}. This is before the previously received IMU message that occurred at: {self.imu_queue[-1].header.stamp}. The current IMU queue will be reset.")
            self.imu_queue.clear()

        self.imu_queue.append(msg)

        if len(self.imu_queue) > self.max_imu_queue_length:
            self.imu_queue.pop(0)
        
        try:
            self.integrate_imu_data(msg)
        except Exception as e:
            self.get_logger().error(f"IMU INTEGRATION FAILED, RESETTING EVERYTHING: {str(e)}")
            self.have_bias = False
            self.have_odom = False
            self.imu_queue.clear()
            return

        self.publish_odometry()
        self.publish_tf()
        self.seq += 1

    def imu_bias_callback(self, msg):
        self.imu_linear_acceleration_bias = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.imu_angular_velocity_bias = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.have_bias = True

    def integrate_imu_data(self, msg):
        if not self.has_imu_meas:
            self.estimate_timestamp = msg.header.stamp
            self.has_imu_meas = True
            return
        
        delta_time = (msg.header.stamp - self.estimate_timestamp).to_sec()
        
        kGravity = np.array([0.0, 0.0, -9.81])
        
        imu_linear_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        imu_angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        
        final_angular_velocity = imu_angular_velocity - self.imu_angular_velocity_bias
        delta_angle = delta_time * (final_angular_velocity + self.angular_velocity) / 2.0
        self.angular_velocity = final_angular_velocity
        
        half_delta_rotation = self.exp(delta_angle / 2.0)
        
        if not self.have_orientation:
            self.transform[:3, :3] = np.dot(self.transform[:3, :3], half_delta_rotation[:3, :3])
        
        delta_linear_velocity = delta_time * (imu_linear_acceleration + self.transform[:3, :3].T @ kGravity - self.imu_linear_acceleration_bias)
        self.transform[:3, 3] += self.transform[:3, :3] @ (delta_time * (self.linear_velocity + delta_linear_velocity / 2.0))
        self.linear_velocity += delta_linear_velocity
        
        if not self.have_orientation:
            self.transform[:3, :3] = np.dot(self.transform[:3, :3], half_delta_rotation[:3, :3])
        
        self.estimate_timestamp = msg.header.stamp

    def publish_odometry(self):
        msg = Odometry()
        msg.header.frame_id = self.frame_id
        msg.header.seq = self.seq
        msg.header.stamp = self.estimate_timestamp
        msg.child_frame_id = self.child_frame_id
        
        # Fill in the Odometry message fields here
        # For example:
        msg.pose.pose.position.x = self.transform[0, 3]
        msg.pose.pose.position.y = self.transform[1, 3]
        msg.pose.pose.position.z = self.transform[2, 3]
        
        # Fill in the orientation here
        
        msg.twist.twist.linear.x = self.linear_velocity[0]
        msg.twist.twist.linear.y = self.linear_velocity[1]
        msg.twist.twist.linear.z = self.linear_velocity[2]
        
        msg.twist.twist.angular.x = self.angular_velocity[0]
        msg.twist.twist.angular.y = self.angular_velocity[1]
        msg.twist.twist.angular.z = self.angular_velocity[2]
        
        self.odom_pub.publish(msg)

    def publish_tf(self):
        msg = TransformStamped()
        msg.header.frame_id = self.frame_id
        msg.header.seq = self.seq
        msg.header.stamp = self.estimate_timestamp
        msg.child_frame_id = self.child_frame_id
        
        # Fill in the TransformStamped message fields here
        # For example:
        msg.transform.translation.x = self.transform[0, 3]
        msg.transform.translation.y = self.transform[1, 3]
        msg.transform.translation.z = self.transform[2, 3]
        
        # Fill in the rotation here
        
        self.transform_pub.publish(msg)
        self.br.sendTransform(msg)

    def quaternion_to_matrix(self, quaternion):
        # Convert a quaternion to a rotation matrix
        pass
    
    def exp(self, vector):
        # Compute the matrix exponential of a vector
        pass

def main(args=None):
    rclpy.init(args=args)
    node = OdomPredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
