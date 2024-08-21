#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_messages.msg import MotorCommand
from geometry_msgs.msg import Twist


class MecanumPublisher(Node):
    def __init__(self):
        super().__init__('mecanum_publisher')
        self.publisher_ = self.create_publisher(MotorCommand, '/publish_motor', 10)
        self.subscription_ = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

        self.Lx = 23    # distance between the two wheels on the x-axis (Dx = 43cm)
        self.Ly = 20    # distance between the two wheels on the y-axis (Dy = 40cm)
        self.r = 15.3/2  # radius of the wheel (D = 15.3cm)
        self.motors = [0, 0, 0, 0]  # motor speeds

    def map_value(self, value, from_min, from_max, to_min, to_max):
        # Map the value from one range to another
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min


    def listener_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        w = msg.angular.z

        w = self.map_value(w, -80.0, 80.0, -1.0, 1.0)

        # w = w * 0.001

        self.move_robot(vx, vy, w)
        self.get_logger().info('Publishing: "%s"' % self.motors)

    def calculate_mecanum_motors(self, vx, vy, w):
        self.motors[0] = (vx - vy - (self.Lx + self.Ly) * w) * 1/ self.r
        self.motors[1] = (vx + vy + (self.Lx + self.Ly) * w) * 1/ self.r
        self.motors[2] = (vx + vy - (self.Lx + self.Ly) * w) * 1/ self.r
        self.motors[3] = (vx - vy + (self.Lx + self.Ly) * w) * 1/ self.r

        # M[0] = (Vx - Vy - (lx + ly) * omega) * 1 / r;
        # M[1] = (Vx + Vy + (lx + ly) * omega) * 1 / r;
        # M[2] = (Vx - Vy + (lx + ly) * omega) * 1 / r;
        # M[3] = (Vx + Vy - (lx + ly) * omega) * 1 / r;

        return self.motors        
    
    def move_robot(self, vx, vy, w):

        max_speed = 50
        min_speed = -50
    
        self.motors = [max(min(motor, max_speed), min_speed) for motor in self.motors]

        self.calculate_mecanum_motors(vx, vy, w)
        for i in range(4):
            msg = MotorCommand()
            msg.motor_id = i + 1
            msg.goal = self.motors[i]
            msg.speedmode = True
            msg.stop = False
            msg.reset = False
            msg.voltagemode = False
            self.publisher_.publish(msg)
            # self.get_logger().info('Publishing: "%s"' % msg)
        
      

def main(args=None):
    rclpy.init(args=args)

    mecanum_publisher = MecanumPublisher()

    rclpy.spin(mecanum_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mecanum_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()