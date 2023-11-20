#!/usr/bin/env python3
#ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
#Python libraries
import serial
import math

class SerialPortReader(Node):
    def __init__(self):
        super().__init__("serial_port_reader")

        self.angle_in_radians_pub = self.create_publisher(Float32, "/vonns2/serial_port/angle_in_radians", 10)
        self.angle_in_radians_msg = Float32()
        
        self.angle_in_degrees_pub = self.create_publisher(Int32, "/vonns2/serial_port/angle_in_degrees", 10)
        self.angle_in_degrees_msg = Int32()

        self.angle_in_degrees = 0
        self.angle_in_radians = 0

        # Serial port's information
        self.microcontroller_port = "/dev/ttyUSB0"
        self.microcontroller_baudrate = 115200
        self.serial_port = serial.Serial(
            self.microcontroller_port, 
            self.microcontroller_baudrate, 
            timeout=10
        )
        self.read_from_serial_port()

    def read_from_serial_port(self):
        while True:
            # Reading serial port data
            angle_byte = self.serial_port.readline()
            angle_string = angle_byte.decode("utf-8").strip('\n').strip('\r') # Decode serial port data
            print(angle_string)
            
            # Transforming serial port data into Int and Float datatype
            self.angle_in_degrees = int(angle_string) # Int datatype
            self.angle_in_radians = math.radians(self.angle_in_degrees) # Float datatype

            # Publishing angle in degrees
            self.angle_in_degrees_msg.data = self.angle_in_degrees
            self.angle_in_degrees_pub.publish(self.angle_in_degrees_msg)

            # Publishing angle in radians
            self.angle_in_radians_msg.data = self.angle_in_radians
            self.angle_in_radians_pub.publish(self.angle_in_radians_msg)

def main():
    rclpy.init()
    serial_port_reader_node = SerialPortReader()
    try:
        rclpy.spin(serial_port_reader_node)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == "__main__":
    main()