#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from custom_messages.msg import MotorCommand
import can
import subprocess
import struct

class MotorCommandNode(Node):

    def __init__(self):
        super().__init__('motor_command_node')
        
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

        self.subscriber = self.create_subscription(MotorCommand, '/publish_motor', self.callback, 1000)
        
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
        
        self.canMsgData = [0, 0, 0, 0, 0, 0, 0, 0]
        

    def callback(self, msg):
        self.canMsgData[0] = (
            msg.speedmode +
            (msg.stop << 1) +
            (msg.reset << 2) +
            (msg.voltagemode << 3)
        )
        
        goal_bytes = struct.pack("<f", msg.goal)
    	
        self.canMsgData[2:6] = goal_bytes
        
        can_msg = can.Message(arbitration_id=msg.motor_id, data=self.canMsgData, is_extended_id=False)
        try:
            self.bus.send(can_msg)
            print("Message sent on {}".format(self.bus.channel_info))
        except can.CanError:
            print("Message NOT sent")
        
    

def main(args=None):
    rclpy.init(args=args)

    motor_command_node = MotorCommandNode()

    rclpy.spin(motor_command_node)

    motor_command_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
