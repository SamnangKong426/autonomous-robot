#!/usr/bin/env python3
import serial
import struct
import math
import serial.tools.list_ports
from sensor_msgs.msg import Imu, MagneticField
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion

def checkSum(list_data, check_data):
    data = bytearray(list_data)
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])


# Hexadecimal to IEEE floating point
def hex_to_ieee(raw_data):
    ieee_data = []
    raw_data.reverse()
    for i in range(0, len(raw_data), 4):
        data2str = hex(raw_data[i] | 0xff00)[4:6] + hex(raw_data[i + 1] | 0xff00)[4:6] + hex(raw_data[i + 2] | 0xff00)[4:6] + hex(raw_data[i + 3] | 0xff00)[4:6]
        ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
    ieee_data.reverse()
    return ieee_data

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        # self.mag_publisher = self.create_publisher(MagneticField, 'imu/mag', 10)
        #self.timer = self.create_timer(1.0, self.timer_callback)

        self.key = 0
        self.flag = 0
        self.buff = {}
        self.angularVelocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.pub_flag= [True, True]
        self.data_right_count = 0

        self.gra_normalization = True

        self.setup_serial_port()

        self.timer = self.create_timer(0.0033, self.read_from_serial_port)
        self.get_logger().info("imu is running successfully")


    def setup_serial_port(self):
        try:
            # set serial port
            self.hf_imu = serial.Serial(port="/dev/ttyUSB0", baudrate=921600, timeout=0.5)
            if self.hf_imu.isOpen():
                self.get_logger().info("Serial port opened successfully")
            else:
                self.hf_imu.open()
                self.get_logger().info("Opened serial port")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.destroy_node()
            exit(0)

    def read_from_serial_port(self):
        if self.hf_imu.inWaiting() > 0:
            buff_data = self.hf_imu.read(self.hf_imu.inWaiting())
            for byte_data in buff_data:
                self.handleSerialData(byte_data)

    def handleSerialData(self, raw_data):
        if self.data_right_count > 200000:
            print("The device has data transmission errors, exiting")
            exit(0)

        self.buff[self.key] = raw_data

        self.key += 1
        if self.buff[0] != 0xaa:
            self.data_right_count += 1
            self.key = 0
            return
        if self.key < 3:
            return
        if self.buff[1] != 0x55:
            self.key = 0
            return
        if self.key < self.buff[2] + 5:  # Determine the corresponding length of data based on the data length byte
            return

        else:
            self.data_right_count = 0
            data_buff = list(self.buff.values())  # Get all values of the dictionary

            if self.buff[2] == 0x2c and self.pub_flag[0]:
                if checkSum(data_buff[2:47], data_buff[47:49]):
                    data = hex_to_ieee(data_buff[7:47])
                    self.angularVelocity = data[1:4]
                    self.acceleration = data[4:7]
                    self.magnetometer = data[7:10]
                else:
                    print('Checksum failed')
                self.pub_flag[0] = False
            elif self.buff[2] == 0x14 and self.pub_flag[1]:
                if checkSum(data_buff[2:23], data_buff[23:25]):
                    data = hex_to_ieee(data_buff[7:23])
                    self.angle_degree = data[1:4]
                else:
                    print('Checksum failed')
                self.pub_flag[1] = False
            else:
                print("The data handler does not provide a parser for " + str(self.buff[2]) + " or the data is incorrect")
                self.buff = {}
                self.key = 0

            self.buff = {}
            self.key = 0
            # if self.pub_flag[0] == True or self.pub_flag[1] == True:
            #    return
            self.pub_flag[0] = self.pub_flag[1] = True
            stamp = self.get_clock().now().to_msg()

            imu_msg = Imu()

            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = "bno055"

            mag_msg = MagneticField()

            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = "bno055"

            angle_radian = [self.angle_degree[i] * math.pi / 180 for i in range(3)]
            qua = quaternion_from_euler(angle_radian[0], -angle_radian[1], -angle_radian[2])

            imu_msg.orientation.x = qua.x
            imu_msg.orientation.y = qua.y
            imu_msg.orientation.z = qua.z
            # imu_msg.orientation.w = angle_radian[2]
            imu_msg.orientation.w = qua.w

            imu_msg.angular_velocity.x = self.angularVelocity[0]
            imu_msg.angular_velocity.y = self.angularVelocity[1]
            imu_msg.angular_velocity.z = self.angularVelocity[2]

            acc_k = math.sqrt(self.acceleration[0] ** 2 + self.acceleration[1] ** 2 + self.acceleration[2] ** 2)
            if acc_k == 0:
                acc_k = 1

            if self.gra_normalization:
                imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8 / acc_k
                imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8 / acc_k
                imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8 / acc_k
            else:
                imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8
                imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8
                imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8

            mag_msg.magnetic_field.x = self.magnetometer[0]
            mag_msg.magnetic_field.y = self.magnetometer[1]
            mag_msg.magnetic_field.z = self.magnetometer[2]

            self.imu_publisher.publish(imu_msg)
            # self.mag_publisher.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()