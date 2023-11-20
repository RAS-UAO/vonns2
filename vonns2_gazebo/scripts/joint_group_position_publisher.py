#!/usr/bin/env python3
#ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray
#Python libraries
import math

class JointGroupPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_group_position_publisher')

        self.current_position_in_degrees = 0.0
        self.current_position_in_radians = math.radians(self.current_position_in_degrees)
        self.declare_parameter("max_rotation_angle_in_degrees", 35)
        self.final_position_in_degrees = self.get_parameter("max_rotation_angle_in_degrees").get_parameter_value().integer_value
        self.final_position_in_radians = math.radians(self.final_position_in_degrees)
        
        self.joint_group_position_pub = self.create_publisher(
             Float64MultiArray, 
             "/joint_group_position_controller/commands", 
             10
        )

        self.angle_in_degrees_publisher = self.create_publisher(
            Int32,
            "/vonns2/serial_port/angle_in_degrees",
            10
        )

        self.angle_in_degrees_msg = Int32()

        self.joint_group_position_msg = Float64MultiArray()
        self.joint_group_position_timer = self.create_timer(1.0, self.publish_position_in_joints)

    def publish_position_in_joints(self):
        self.current_position_in_degrees += 1
        self.current_position_in_radians = math.radians(self.current_position_in_degrees)

        if(self.current_position_in_radians >= self.final_position_in_radians):
            self.current_position_in_radians = self.final_position_in_radians
            self.current_position_in_degrees = math.degrees(self.final_position_in_radians)

        self.joint_group_position_msg.data = [
            self.current_position_in_radians
        ]
        self.joint_group_position_pub.publish(self.joint_group_position_msg)
        
        self.angle_in_degrees_msg.data = int(self.current_position_in_degrees)
        self.angle_in_degrees_publisher.publish(self.angle_in_degrees_msg)

        print("\nPosition in Joint Group:")
        print(f"servo_joint: {self.current_position_in_degrees} °\n")

def main(args=None):
    rclpy.init(args=args)
    joint_group_position_pub_node = JointGroupPositionPublisher()
    try:
        rclpy.spin(joint_group_position_pub_node)
    except KeyboardInterrupt:
        joint_group_position_pub_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()