#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import math
from sensor_msgs.msg import JointState

arduino_port = "/dev/ttyUSB0"

class TranformBroadcaster(Node):
    def __init__(self):
        super().__init__("transform_broadcaster")

        self.parent_frame = "base_link"
        self.child_frame = "laser"
        self.laser_joint_publisher = self.create_publisher(JointState, "/joint_states", 10)

        self.angle_in_grades = 0
        self.angle_in_radians = 0
        self.time_period = 1 #segundos
        self.timer = self.create_timer(self.time_period, self.get_angle)

    def send_transform_in_urdf(self, angle_in_radians):
        laser_joint_msg = JointState()
        laser_joint_msg.header.stamp = self.get_clock().now().to_msg()
        laser_joint_msg.header.frame_id = self.child_frame
        laser_joint_msg.name = ['laser_joint']
        laser_joint_msg.position = [float(angle_in_radians)]

        self.laser_joint_publisher.publish(laser_joint_msg)

    def get_angle(self, serial_port):
        angle_byte = serial_port.readline() #Se lee el angulo del Arduino

        try:
            angle_string = angle_byte.decode("utf-8").strip('\n').strip('\r') #Se transforma el angulo en entero
            self.angle_in_grades = int(angle_string)
            print(f"Angle: {self.angle_in_grades}°")
            self.angle_in_radians = float(math.radians(self.angle_in_grades))
        except:
            pass

        self.send_transform_in_urdf(self.angle_in_radians)

        if (self.angle_in_grades > 45):
            return

def main():
    rclpy.init()
    my_node = TranformBroadcaster()
    try:
        with serial.Serial(arduino_port, 9600, timeout=1) as serial_port:
            while rclpy.ok:
                my_node.get_angle(serial_port)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == "__main__":
    main()