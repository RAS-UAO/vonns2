//Librerias de C++
#include <iostream>
//Librerias de ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
//Librerias adicionales
#include <math.h>

class TransformBroadcaster: public rclcpp::Node{
    public:
        TransformBroadcaster(): Node("transform_broadcaster"){
            std::cout << "Start subscribing to angle publishing!" << std::endl;

            angle_sub = this->create_subscription<std_msgs::msg::Float32>(
                "/vonns2/serial_port/angle_in_radians",
                10,
                std::bind(&TransformBroadcaster::angle_callback, this, std::placeholders::_1)
            );
            
            laser_joint_pub = this->create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states",
                10
            );
        }
    private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr laser_joint_pub;
        float angle_in_radians = 0;
        std::string child_frame = "laser";

        void send_transform_in_urdf(float angle_in_radians){
            auto laser_joint_msg = sensor_msgs::msg::JointState();
            laser_joint_msg.header.stamp = rclcpp::Clock().now();
            laser_joint_msg.header.frame_id = child_frame;
            laser_joint_msg.name = {"servo_joint"};
            laser_joint_msg.position = {angle_in_radians};

            laser_joint_pub->publish(laser_joint_msg);
        }

        void angle_callback(std_msgs::msg::Float32::SharedPtr angle_msg){
            angle_in_radians = angle_msg->data;
            this->send_transform_in_urdf(angle_in_radians);
        }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    
    auto transformer_broadcaster_node = std::make_shared<TransformBroadcaster>();
    rclcpp::spin(transformer_broadcaster_node);

    rclcpp::shutdown();
    return 0;
}