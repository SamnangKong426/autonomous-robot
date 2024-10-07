#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can
from custom_messages.msg import MotorFeedback
import struct
import subprocess

class MotorFeedbackNode(Node):

    def __init__(self):
        super().__init__('motor_feedback_node')
        cmd = ["ip", "link", "show", "can0"]
        
        result = subprocess.run(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        
        if b"state UP" in result.stdout:
            print("CAN interface is already up")
        else:
            cmd = ["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "1000000"]
            
            result = subprocess.run(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
            
            if result.returncode == 0:
                print("CAN interface is up")
            else:
                print("CAN failed to setup")
                
        self.publisher = self.create_publisher(MotorFeedback, '/motor_feedback', 10)
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
        self.listener = CanListener(self.publisher)
        self.notifier = can.Notifier(self.bus, [self.listener])

class CanListener(can.Listener):
    def __init__(self, publisher):
        self.publisher = publisher

    def on_message_received(self, msg):
        # process received message and publish to /motor_feedback topic
        if (msg.arbitration_id > 128 and msg.arbitration_id < 256):
            feedback_msg = MotorFeedback()
            feedback_msg.motor_id = msg.arbitration_id
            feedback_msg.position = struct.unpack_from("<f", msg.data, 0)[0]
            feedback_msg.speed = struct.unpack_from("<f", msg.data, 4)[0]
            self.publisher.publish(feedback_msg)

def main(args=None):
    rclpy.init(args=args)
    motor_feedback_node = MotorFeedbackNode()
    rclpy.spin(motor_feedback_node)
    motor_feedback_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
