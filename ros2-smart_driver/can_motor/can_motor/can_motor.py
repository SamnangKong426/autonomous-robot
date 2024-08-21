#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_messages.msg import MotorFeedback, MotorCommand, CanMsg
import struct

class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')
        self.encoder_can_listener = self.create_subscription(CanMsg, 'can_receive', self.encoder_callback, 10)
        self.motor_publish_listener = self.create_subscription(MotorCommand, '/publish_motor', self.motor_publish_callback, 10)
        self.encoder_publisher = self.create_publisher(MotorFeedback, 'motor_feedback', 10)
        self.motor_publisher = self.create_publisher(CanMsg, 'transmit_can', 10)
        self.can_msg_to_send = CanMsg()

    def encoder_callback(self, msg):
        if 128 < msg.can_id < 256:
            feedback_msg = MotorFeedback()
            feedback_msg.motor_id = msg.can_id
            feedback_msg.position = struct.unpack_from("<f", msg.data, 0)[0]
            feedback_msg.speed = struct.unpack_from("<f", msg.data, 4)[0]
            self.encoder_publisher.publish(feedback_msg)

    def motor_publish_callback(self, msg):
        self.can_msg_to_send.can_id = msg.motor_id
        self.can_msg_to_send.data[0] = (
            msg.speedmode +
            (msg.stop << 1) +
            (msg.reset << 2) +
            (msg.voltagemode << 3)
        )

        goal_bytes = struct.pack("<f", msg.goal)

        # Corrected line
        self.can_msg_to_send.data[2:6] = list(goal_bytes)
        self.motor_publisher.publish(self.can_msg_to_send)

def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorNode()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

