#include <rclcpp/rclcpp.hpp>  
#include <sensor_msgs/msg/point_cloud2.hpp>  
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl/conversions.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include <random>  

class RandomPointCloudPublisher : public rclcpp::Node {  
public:  
    RandomPointCloudPublisher() : Node("random_pointcloud_publisher") {  
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("random_points", 10);  
        timer_ = this->create_wall_timer(  
            std::chrono::milliseconds(500),  
            std::bind(&RandomPointCloudPublisher::publish_point_cloud, this)  
        );  
    }  

private:  
    void publish_point_cloud() {  
        pcl::PointCloud<pcl::PointXYZ> cloud;  
        cloud.width = 100; // 点数量  
        cloud.height = 1;  // 单个帧  
        cloud.points.resize(cloud.width * cloud.height);  

        // 使用随机数生成器生成随机点  
        std::default_random_engine generator;  
        std::uniform_real_distribution<float> distribution(0.0f, 1024.0f);  

        // 生成随机点  
        for (auto &point : cloud.points) {  
            point.x = distribution(generator);  
            point.y = distribution(generator);  
            point.z = distribution(generator);  
        }  

        sensor_msgs::msg::PointCloud2 cloud_msg;  
        pcl::toROSMsg(cloud, cloud_msg);  
        cloud_msg.header.frame_id = "map"; // 设置 frame_id  
        cloud_msg.header.stamp = this->get_clock()->now(); // 设置时间戳  

        publisher_->publish(cloud_msg);  
    }  

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;  
    rclcpp::TimerBase::SharedPtr timer_;  
};  

int main(int argc, char **argv) {  
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make_shared<RandomPointCloudPublisher>());  
    rclcpp::shutdown();  
    return 0;  
}