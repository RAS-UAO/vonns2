//Librerias de C++
#include <iostream>
//Librerias de ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//Librerias de PCL
#include <pcl/common/common.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>


class PointCloudSubscriber: public rclcpp::Node{
    public: 
        PointCloudSubscriber(): Node("point_cloud2_subscriber"){
            std::cout << "PointCloud2 Subscriber Started!" << std::endl;

            point_cloud2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/vonns2/point_cloud2",
                10,
                std::bind(&PointCloudSubscriber::point_cloud2_callback, this, std::placeholders::_1)
            );

            angle_sub = this->create_subscription<std_msgs::msg::Int32>(
                "/vonns2/serial_port/angle_in_degrees",
                10,
                std::bind(&PointCloudSubscriber::angle_callback, this, std::placeholders::_1)
            );

            initializing_empty_concatenated_pcl();
        }
    private:
        //Nube de Puntos
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_sub; //Suscriptor de Nube de Puntos
        pcl::PointCloud<pcl::PointXYZ> concatenated_cloud; //Nube de Puntos Final

        //Angulo
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr angle_sub; //Suscriptor del Angulo
        int angle_in_degrees = 0; //Angulo en Grados
        int superior_limit_angle = 60; //Angulo Maximo en Grados

        //Inicializando la Nube de Puntos Total
        void initializing_empty_concatenated_pcl(){
            pcl::PointCloud<pcl::PointXYZ> empty_cloud;
            empty_cloud.height = 0;
            empty_cloud.width = 0;
            empty_cloud.is_dense = true;

            concatenated_cloud = empty_cloud;
        }
        
        //Añadiendo Escaneos a la Nube de Puntos Total
        void concatenate_current_pcl_to_total_cloud(pcl::PointCloud<pcl::PointXYZ> &current_cloud){
            concatenated_cloud += current_cloud;
        } 

        //Convertir Nube de Puntos de PointCloud2 a ROS PCL
        pcl::PointCloud<pcl::PointXYZ> convert_from_ros2_to_pcl(sensor_msgs::msg::PointCloud2::SharedPtr ros_cloud_msg){
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_msg(new pcl::PointCloud<pcl::PointXYZ>); 
            pcl::fromROSMsg(*ros_cloud_msg, *pcl_cloud_msg);
            pcl::PointCloud<pcl::PointXYZ> &new_cloud = *pcl_cloud_msg;
            return new_cloud;
        }

        void point_cloud2_callback(sensor_msgs::msg::PointCloud2::SharedPtr ros_cloud_msg){
            if (angle_in_degrees == 0){
                return;
            }
            else{
                pcl::PointCloud<pcl::PointXYZ> current_cloud;
                current_cloud = convert_from_ros2_to_pcl(ros_cloud_msg);
                concatenate_current_pcl_to_total_cloud(current_cloud);
            }
        }

        //Filtrar Nube de Puntos de PCD
        void apply_sor_filter_to_pcd(){
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PCDReader reader;
            const char *input_file_path = "/home/usuario/Workspaces/personal_ws/src/vonns2/pcd/reconstruction.pcd";
            reader.read<pcl::PointXYZ>(input_file_path, *input_cloud);

            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier_removal_filter;
            statistical_outlier_removal_filter.setInputCloud(input_cloud);
            statistical_outlier_removal_filter.setMeanK(15);
            statistical_outlier_removal_filter.setStddevMulThresh(0.5);
            statistical_outlier_removal_filter.setNegative (true);
            statistical_outlier_removal_filter.filter(*cloud_filtered);
            
            pcl::PCDWriter writer;
            const char *outlier_file_path = "/home/usuario/Workspaces/personal_ws/src/vonns2/pcd/reconstruction_filtered.pcd";
            writer.write<pcl::PointXYZ>(outlier_file_path, *cloud_filtered, false);
        }

        //Convertir Nube de Puntos a PCD
        void convert_total_cloud_to_pcd(){
            pcl::io::savePCDFileASCII ("/home/usuario/Workspaces/personal_ws/src/vonns2/pcd/reconstruction.pcd", concatenated_cloud);
        }

        void angle_callback(std_msgs::msg::Int32::SharedPtr angle_msg){
            angle_in_degrees = angle_msg->data;
            printf("LiDAR Angle: %i°\n", angle_in_degrees);
            if(angle_in_degrees >= superior_limit_angle){
                convert_total_cloud_to_pcd();
                std::cerr << "PCD File Saved!" << std::endl;

                apply_sor_filter_to_pcd();
                std::cerr << "PCD Filtered!" << std::endl;
            }
        }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    
    auto pointcloud2_subscriber_node = std::make_shared<PointCloudSubscriber>();
    rclcpp::spin(pointcloud2_subscriber_node);

    rclcpp::shutdown();
    return (0);
}
