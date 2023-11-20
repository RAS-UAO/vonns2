// C++ Libraries
#include <iostream>
#include <string>
#include <deque>
// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "laser_geometry/laser_geometry.hpp"
#include "filters/filter_chain.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "boost/thread.hpp"

class LaserScanAssembler: public rclcpp::Node{
    public:
        LaserScanAssembler(): Node("laser_scan_asssembler"), filter_chain("sensor_msgs::msg::LaserScan"){
            std::cerr << "Starting laser_scan_asssembler node..." << std::endl;

            this->declare_parameter("ignore_laser_skew", true);
            ignore_laser_skew = this->get_parameter("ignore_laser_skew").as_bool();
            
            // Getting fixed frame id for final cloud
            this->declare_parameter("fixed_frame", "base_link");
            fixed_frame_id = this->get_parameter("fixed_frame").as_string();
            assembled_cloud.header.frame_id = fixed_frame_id;

            // Getting maximum angle for rotation
            this->declare_parameter("max_rotation_angle_in_degrees", 35);
            max_angle_in_degrees = this->get_parameter("max_rotation_angle_in_degrees").as_int();

            // Passing in the parameter namespace to read from the Parameter Server 
            filter_chain.configure("filters", node_logger, node_params);

            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            angle_sub = this->create_subscription<std_msgs::msg::Int32>(
                "/vonns2/serial_port/angle_in_degrees",
                10,
                std::bind(&LaserScanAssembler::angle_callback, this, std::placeholders::_1)
            );

            laser_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(&LaserScanAssembler::scan_callback, this, std::placeholders::_1)
            );

            point_cloud2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/vonns2/point_cloud2", 
                10
            );

            assembled_point_cloud2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/vonns2/assembled_point_cloud2", 
                10
            );

            std::cerr << "Ready to assemble PointCloud2..." << std::endl;
        }
        
        void angle_callback(std_msgs::msg::Int32::SharedPtr angle_msg){
            angle_in_degrees = angle_msg->data;
        }

        // Generating final PointCloud2
        void assemble_point_cloud2(){
            // Add the new PointCloud into history of LaserScans
            point_cloud2_history_mutex.lock();
            
            // Maintaining a history of all previous LaserScans
            point_cloud2_history.push_back(current_cloud);

            // Defining assembled PointCloud2's size
            total_points_in_assembled_cloud += current_cloud.data.size();

            for (const auto& point_cloud : point_cloud2_history) {
                if (assembled_cloud.data.empty()) {
                    assembled_cloud = point_cloud;
                } 
                else {
                    assembled_cloud.data.insert(
                        assembled_cloud.data.end(),
                        point_cloud.data.begin(),
                        point_cloud.data.end()
                    );
                    assembled_cloud.header.stamp = this->now();
                }
            }

            point_cloud2_history_mutex.unlock();

            return;
        }

        // Convert the LIDAR's laser scans to point cloud
        void convert_laser_scan_to_point_cloud2(const std::string& fixed_frame_id, sensor_msgs::msg::LaserScan& scan_in, sensor_msgs::msg::PointCloud2& cloud_out){
            // Faster (approximate) way to make conversion
            if (ignore_laser_skew == true){
                laser_projector.projectLaser(scan_in, cloud_out);

                // Changing PointCloud2's frame ID if needed
                if (cloud_out.header.frame_id != fixed_frame_id){
                    try {
                        // Computing transformation between specified frames in specified time
                        transform_stamped = tf_buffer->lookupTransform(
                            fixed_frame_id, // Fixed frame
                            cloud_out.header.frame_id, // PointCloud2's current frame
                            cloud_out.header.stamp, // PointCloud2's current time
                            rclcpp::Duration::from_seconds(2.5) // Waiting time for transformations computing up to 1 second 
                        ); 
                        
                        // Transforming PointCloud2 based on previus transformation computing
                        tf2::doTransform(cloud_out, cloud_out, transform_stamped);
                    }
                    catch (tf2::TransformException &ex) {
                        std::cerr << "Transform not computed" << std::endl;
                    }
                }
            }
            return;
        }

        // Subscribing to the LaserScans sended by LiDAR or similar-type devices
        void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg){
            std::cerr << "\nIncomming data from LiDAR: " << std::endl;

            if(angle_in_degrees == 0){
                std::cerr << "LaserScan message not processed at " << angle_in_degrees << " °!" << std::endl;
                return;
            }
            else{
                if(angle_in_degrees >= max_angle_in_degrees){
                    std::cerr << "Final PointCloud2 reached at " << max_angle_in_degrees << " °!" << std::endl;
                    return;
                }

                sensor_msgs::msg::LaserScan scan_in = *laser_scan_msg;
                std::cerr << "LaserScan message from LiDAR received at " << angle_in_degrees << " °!" << std::endl;
            
                sensor_msgs::msg::LaserScan scan_filtered;
                filter_chain.update(scan_in, scan_filtered);
                std::cerr << "LaserScan message from LiDAR filtered at " << angle_in_degrees << " °!" << std::endl;
                
                if (ignore_laser_skew != true){
                    tf_filter->add(laser_scan_msg);
                }

                // Converting LaserScan to PointCloud2 for PointCloud2 assembling
                convert_laser_scan_to_point_cloud2(fixed_frame_id, scan_filtered, current_cloud);
                point_cloud2_pub->publish(current_cloud);
                std::cerr << "LaserScan message converted into current PointCloud2!" << std::endl;

                // Assembling new PointCloud2
                
                if (assembled_cloud.data.empty()) {
                    assembled_cloud = current_cloud;
                } 
                else {
                    assembled_cloud.data.insert(
                        assembled_cloud.data.end(),
                        current_cloud.data.begin(),
                        current_cloud.data.end()
                    );
                }
                assembled_point_cloud2_pub->publish(assembled_cloud);
                std::cerr << "Current PointCloud2 assembled!" << std::endl;
            }
        }

    private:
        // Rotation angle
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr angle_sub;
        int angle_in_degrees = 0; // Rotation angle expressed in degrees
        int max_angle_in_degrees; // Maximum rotation angle expressed in degrees

        // Sequence of filters for LaserScan data processing
        filters::FilterChain<sensor_msgs::msg::LaserScan> filter_chain; // Dynamically loading a sequence of Filters based on runtime parameters
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logger = this->get_node_logging_interface();
        rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params = this->get_node_parameters_interface();

        // Ways to compute transformations
        geometry_msgs::msg::TransformStamped transform_stamped;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        // Accumulation of PointCloud2 based on LaserScan
        bool ignore_laser_skew;
        std::string fixed_frame_id;
        tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>* tf_filter;
        boost::mutex point_cloud2_history_mutex;
        std::deque<sensor_msgs::msg::PointCloud2> point_cloud2_history;
        sensor_msgs::msg::PointCloud2 current_cloud;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub;

        // Way to convert LaserScan to PointCloud2
        laser_geometry::LaserProjection laser_projector;
        int max_scans = 400 ;
        sensor_msgs::msg::PointCloud2 assembled_cloud;
        unsigned int total_points_in_assembled_cloud;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr assembled_point_cloud2_pub;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto laser_scan_assembler_node = std::make_shared<LaserScanAssembler>();
    rclcpp::spin(laser_scan_assembler_node);

    rclcpp::shutdown();
    return 0;
}
